# Car day: the dated records

Moved from [Car day](../car-day.md) on 18 Nov (TODO 172: shorter pages), word for word, so
that page stays the steps.

### `pixi run car-day-check` on Ben's Mac, 24 Oct

After step 4, before steps 1, 5 and 6: so the repo, the build and the kit are its 3
missing, and Cyclone a reminder. (The repo line's advice is today's wording.) No secrets: the Keychain is only asked whether each
entry exists (on a laptop without them, the vault holds them).

```text
Car day: is /Users/ben/Code/ecolo-av ready? (car-day.md)

  MISSING  repo         main at 54304074 has commits GitHub hasn't
                        -> pixi run test, then move them to a branch and open a pull request (CONTRIBUTING.md)
  MISSING  build        ros/ changed since the last build (24 Oct 15:06)
                        -> pixi run build
  MISSING  car kit      made 2026-10-20 13:54; its code is 37a7b549, main is 54304074
                        -> pixi run car-kit
  REMIND   USB stick    no stick with car-kit/ plugged in
                        -> cp -R car-kit /Volumes/<stick>/
  OK       model        models/rf-detr-nano.pth (366 MB): the detector (RF-DETR Nano): the Orin builds its engine from it
  OK       model        models/yolo11n.pt (6 MB): the YOLO fallback (detector:=yolo)
  OK       model        models/rf-detr-nano.onnx (108 MB): the Orin's engine needs only TensorRT from it (trtexec)
  OK       MediaMTX     vendor/mediamtx (to test a stream on the laptop)
  OK       ESP32 tools  Xtensa Rust, espflash (vendor/esp, vendor/cargo-tools)
  OK       ESP32 Rust   v2 and v1 built 24 Oct 2026 15:47, in car-day/esp32/
  OK       ESP32 QEMU   esp qemu-check passed 24 Oct 2026 15:47: brakes when the heartbeat stops
  OK       ESP32 C++    the 2025-26 C++ builds (the way back), 24 Oct 2026 15:47
  REMIND   ESP32 C++    secrets.h has the placeholders: a C++ built again can't join the WiFi; the backup (session 6) restores it all
                        -> firmware/legacy/aav_umod_esp32/src/secrets.h: the WiFi from the team, typed in, never committed
  OK       layouts      driving (20 topics), map (3 topics), sensors-health (6 topics); checked live at the car (session 8)
  OK       Lichtblick   /Applications/Lichtblick.app
  OK       docs PDF     docs/html/aav-docs.pdf, built 24 Oct 15:48
  REMIND   Cyclone      Cyclone DDS uses one network card: name the car's (the dongle) in ~/.config/aav/cyclonedds-car.xml
                        -> car-day.md, session 0
  OK       logins       7 of 7 in the Keychain: aav-cam2, aav-cortex, aav-jetson, aav-desktop, aav-localization, aav-cam1, aav-qnxpi
  OK       SSH key      ~/.ssh/id_ed25519.pub
  OK       disk         275 GB free for bags
  REMIND   sheet        print measure-the-car.md (2 copies) and a pen
                        -> session 2
  REMIND   kit bag      stands, the RC transmitter, the e-stop, tape, plumb line, 4 scales, blocks, multimeter, dongle, cable
                        -> car-day.md, "What to bring"
  REMIND   questions    the hardware lead is coming? H1-H8 need them
                        -> session 3

14 OK, 3 missing, 6 to tick by hand.
```
