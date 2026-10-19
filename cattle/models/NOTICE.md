# Aerial cattle detector

`aerial_cattle_yolov8m.onnx.gz` is a YOLOv8m detector for cattle in nadir drone photos. Class 0 is cow. Class 1 is background. The sample config keeps class 0.

- Source weights: `models/cow_detection/cow_detection_v2/cow_weight_v2_12.pt` in [ICAERUS-EU/UC3_Livestock_Monitoring](https://github.com/ICAERUS-EU/UC3_Livestock_Monitoring), licensed under GPL-3.0.
- Training images: Helary, L., & Lebreton, A. (2024). *Drone images and their annotations of grazing cows*. Zenodo. https://doi.org/10.5281/zenodo.11048412 (CC-BY-4.0). The photos are nadir Mavic 3 frames from about 60 to 100 m, labeled as 640×640 crops. Reported precision is 0.93 and recall is 0.85.

This file is a derivative of those GPL-3.0 weights. It is distributed under the GNU General Public License, version 3, only. The rest of this Payload SDK repository stays under its own license. A copy of the GPL-3.0 text is `LICENSE.GPL-3.0.txt` in this directory.

## OpenCV 4.6 graph change

Ultralytics exports the box-stride multiply (`/model.22/Mul_2`) with a stride tensor of shape `[1, 8400]`. OpenCV 4.6 cannot broadcast that against the `[1, 4, 8400]` box tensor. The weights are unchanged. The second input of that multiply is an initializer named `cattle_stride_broadcast` with shape `[1, 4, 8400]`: stride 8 on the first 6400 anchors, 16 on the next 1600, and 32 on the last 400, repeated on each of the four box channels.

The `.onnx.gz` package is gzip only. The sample inflates it before loading.
