#!/usr/bin/env python3
"""Checks the cattle email builder without a real mail server."""

import os
import socket
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import send_report


class FakeSmtp(threading.Thread):
    def __init__(self):
        super().__init__()
        self.ready = threading.Event()
        self.message = b""
        self._socket = socket.socket()
        self._socket.bind(("127.0.0.1", 0))
        self.port = self._socket.getsockname()[1]
        self.daemon = True

    def run(self):
        self._socket.listen(1)
        self.ready.set()
        conn, _ = self._socket.accept()
        try:
            self._speak(conn, b"220 test")
            while True:
                line = self._readline(conn)
                if not line:
                    break
                command = line.decode("utf-8", "replace").strip().upper()
                if command.startswith("EHLO") or command.startswith("HELO"):
                    self._speak(conn, b"250 hello")
                elif command.startswith("MAIL FROM"):
                    self._speak(conn, b"250 ok")
                elif command.startswith("RCPT TO"):
                    self._speak(conn, b"250 ok")
                elif command.startswith("DATA"):
                    self._speak(conn, b"354 go")
                    chunks = []
                    while True:
                        data_line = self._readline(conn)
                        if data_line in (b".\r\n", b".\n"):
                            break
                        chunks.append(data_line)
                    self.message = b"".join(chunks)
                    self._speak(conn, b"250 queued")
                elif command.startswith("QUIT"):
                    self._speak(conn, b"221 bye")
                    break
                else:
                    self._speak(conn, b"250 ok")
        finally:
            conn.close()
            self._socket.close()

    @staticmethod
    def _speak(conn, line):
        conn.sendall(line + b"\r\n")

    @staticmethod
    def _readline(conn):
        data = b""
        while b"\n" not in data:
            chunk = conn.recv(1)
            if not chunk:
                break
            data += chunk
        return data


class SendReportTest(unittest.TestCase):
    def test_dry_run_saves_the_count(self):
        with tempfile.TemporaryDirectory() as directory:
            config = os.path.join(directory, "config.json")
            body = os.path.join(directory, "report.txt")
            with open(config, "w", encoding="utf-8") as handle:
                handle.write('{"smtp_host": "", "smtp_port": 587}\n')
            with open(body, "w", encoding="utf-8") as handle:
                handle.write("Cattle count: 7\n")
            status = send_report.main(
                ["--config", config, "--body", body, "--subject", "Cattle count: 7", "--dry-run"]
            )
            self.assertEqual(status, 0)
            eml = os.path.join(directory, "report.eml")
            with open(eml, encoding="utf-8") as handle:
                text = handle.read()
            self.assertIn("Subject: Cattle count: 7", text)
            self.assertIn("Cattle count: 7", text)

    def test_smtp_delivers_the_body(self):
        server = FakeSmtp()
        server.start()
        self.assertTrue(server.ready.wait(2))
        with tempfile.TemporaryDirectory() as directory:
            config = os.path.join(directory, "config.json")
            body = os.path.join(directory, "report.txt")
            with open(config, "w", encoding="utf-8") as handle:
                handle.write(
                    '{"smtp_host": "127.0.0.1", "smtp_port": %d, "smtp_user": "", '
                    '"mail_from": "counter@example.com", "mail_to": "rancher@example.com", '
                    '"starttls": false, "smtp_ssl": false}\n' % server.port
                )
            with open(body, "w", encoding="utf-8") as handle:
                handle.write("Confirmed cattle: 4\n")
            status = send_report.main(
                ["--config", config, "--body", body, "--subject", "Cattle count: 4"]
            )
            self.assertEqual(status, 0)
        server.join(2)
        text = server.message.decode("utf-8", "replace")
        self.assertIn("Subject: Cattle count: 4", text)
        self.assertIn("Confirmed cattle: 4", text)
        self.assertIn("rancher@example.com", text)


if __name__ == "__main__":
    unittest.main()
