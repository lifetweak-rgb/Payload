#!/usr/bin/env python3
"""Email a cattle-count report from the Manifold payload.

SMTP settings live in the same JSON file as the counter. Passwords stay in
that file and are never printed. With an empty smtp_host the message is saved
next to the body as a .eml file so a bench run still produces the report.
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import smtplib
import ssl
import sys
from email.message import EmailMessage


def load_settings(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError("config must be a JSON object")
    return data


def build_message(settings, subject, body, attachment):
    sender = settings.get("mail_from") or settings.get("smtp_user") or ""
    recipient = settings.get("mail_to") or ""
    if not sender or not recipient:
        raise ValueError("mail_from and mail_to are required to send mail")
    message = EmailMessage()
    message["From"] = sender
    message["To"] = recipient
    message["Subject"] = subject
    message.set_content(body)
    if attachment:
        if not os.path.isfile(attachment):
            raise ValueError("attachment not found: %s" % attachment)
        mime, _ = mimetypes.guess_type(attachment)
        if mime is None:
            maintype, subtype = "application", "octet-stream"
        else:
            maintype, subtype = mime.split("/", 1)
        with open(attachment, "rb") as handle:
            payload = handle.read()
        message.add_attachment(
            payload,
            maintype=maintype,
            subtype=subtype,
            filename=os.path.basename(attachment),
        )
    return message


def send_message(settings, message):
    host = settings.get("smtp_host") or ""
    port = int(settings.get("smtp_port") or 587)
    user = settings.get("smtp_user") or ""
    password = settings.get("smtp_password") or ""
    use_ssl = bool(settings.get("smtp_ssl"))
    use_starttls = bool(settings.get("starttls", True)) and not use_ssl
    if use_ssl:
        smtp = smtplib.SMTP_SSL(host, port, timeout=30)
    else:
        smtp = smtplib.SMTP(host, port, timeout=30)
        smtp.ehlo()
        if use_starttls:
            smtp.starttls(context=ssl.create_default_context())
            smtp.ehlo()
    try:
        if user:
            smtp.login(user, password)
        smtp.send_message(message)
    finally:
        try:
            smtp.quit()
        except Exception:
            smtp.close()


def write_eml(message, body_path):
    destination = os.path.splitext(body_path)[0] + ".eml"
    with open(destination, "w", encoding="utf-8") as handle:
        handle.write(message.as_string())
    return destination


def main(argv):
    parser = argparse.ArgumentParser(description="Email a cattle count report")
    parser.add_argument("--config", required=True)
    parser.add_argument("--body", required=True)
    parser.add_argument("--subject", required=True)
    parser.add_argument("--attach", default="")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    settings = load_settings(args.config)
    with open(args.body, "r", encoding="utf-8") as handle:
        body = handle.read()
    dry_run = args.dry_run or not settings.get("smtp_host")
    if dry_run and (not settings.get("mail_from") or not settings.get("mail_to")):
        settings = dict(settings)
        settings.setdefault("mail_from", settings.get("mail_from") or "cattle-counter@localhost")
        settings.setdefault("mail_to", settings.get("mail_to") or "cattle-counter@localhost")
        if not settings.get("mail_from"):
            settings["mail_from"] = "cattle-counter@localhost"
        if not settings.get("mail_to"):
            settings["mail_to"] = "cattle-counter@localhost"

    message = build_message(settings, args.subject, body, args.attach)
    if dry_run:
        path = write_eml(message, args.body)
        print("SMTP host is empty; saved the report to %s" % path)
        return 0
    send_message(settings, message)
    print("Sent cattle count to %s" % message["To"])
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except Exception as exc:
        print("cattle email failed: %s" % exc, file=sys.stderr)
        sys.exit(1)
