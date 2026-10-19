# Cattle counter for the Matrice 4T Manifold

The Manifold 3 sample can watch the Matrice 4T wide camera, keep one identity per animal as the aircraft flies, and email the total.

The wide camera's diagonal field of view is 82°. Each detection is placed on the ground using that field of view, the height above the pasture, and the gimbal angle. A cow seen again a few meters from its last position stays the same animal. The mailer sends the number of animals that were seen at least three times.

## Fly it

1. Copy `cattle/config.example.json` to `cattle/config.json` (or `/open_app/cattle_config.json` on the Manifold).
2. Fill in the SMTP fields. For Gmail, use an app password and `smtp.gmail.com` port 587 with `starttls` true. The Manifold needs a path to the internet (dock Wi-Fi, a dongle, or another uplink) at the moment you send.
3. Build the Manifold 3 C++ sample with OpenCV, the same way as the existing live-view sample.
4. Run it and choose `c`.
5. With the aircraft on the pasture, press `g` to mark the ground altitude. Then take off, point the gimbal straight down, and fly about 60 to 100 m. Stay within about 18° of straight down; tilted frames are ignored.
6. Press `s` for the running total, `e` to email it, or `q` to stop and email.

`camera_source` 1 is the Matrice 4T wide view. Source 3 is the 4K view. The example config already sets `onnx_model` to `models/aerial_cattle_yolov8m.onnx.gz` and `class_ids` to `[0]`, the cow class of that network. A path without a leading `/` is resolved next to the config file. On the Manifold, keep the gzip file at `models/aerial_cattle_yolov8m.onnx.gz` beside the config, or set an absolute path. The first run inflates the archive. If the model cannot be loaded, the sample falls back to the built-in COCO cow class (id 19).

The shipped network is a YOLOv8m detector trained on nadir aerial photos of grazing cows. See `cattle/models/NOTICE.md` for the ICAERUS weights, the Zenodo dataset, and the GPL-3.0 terms that apply to the model file. Each frame is cut into 640-pixel tiles with a 480-pixel stride, and each tile is letterboxed. A second pass letterboxes the whole frame, so a cow still counts when it looks larger than it did in the training photos. The wide view is the one to use; a 4K frame is many more tiles.

The message includes the confirmed count, the unconfirmed sightings, and `data/cattle/latest.jpg` when a frame has been saved. A copy of the text is always written to `data/cattle/cattle_report.txt`. If `smtp_host` is empty, the mailer writes a `.eml` file next to that report instead of sending.

## Check the counter without the aircraft

From the repository root:

```
make -C cattle test
```

That runs the ground-projection tests and a local SMTP delivery test. It does not need the drone, OpenCV, or a mail account.
