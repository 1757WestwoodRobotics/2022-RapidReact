# mentorbot
Code for the FRC 1757 teaching platform robot

## Installation

### Visual Studio 2019 redistributable
[vc_redist.x64](https://aka.ms/vs/16/release/vc_redist.x64.exe)

### Python
[3.9.6 amd64](https://www.python.org/ftp/python/3.9.6/python-3.9.6-amd64.exe)

### VS Code
[VS Code](https://code.visualstudio.com)

### FRC Game Tools
[FRC Game Tools](https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html#369633)

### FRC Radio Configuration Utility
[FRC Configuration Utility](https://firstfrc.blob.core.windows.net/frc2020/Radio/FRC_Radio_Configuration_20_0_0.zip)

### CTRE Phoenix
[Phoenix Tuner](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases)

## Setup

### roboRIO
1. Image the roboRIO
   [Imaging instructions](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/imaging-your-roborio.html)
1. Configure the roboRio
   | Item | Value |
   | - | - |
   | Team number | `1757` |
   | Firmware | `6.0.0f1` |
   | Image | `FRC_roboRIO_2021_v3.0` |
   | Static IP | `10.17.57.2` |
   | Subnet Mask | `255.255.255.0` |

### Run Phoenix Tuner
#### Update device firmware
* PDP
* FalconFX
* CANCoder

#### Configure CAN devices
| Device | Class | Range | ID |
| - | - | - | - |
| robo_rio | core | 0 - 9 | master (no ID) |
| pdp | core | 0 - 9 | 0 |
| front_left_drive | motors | 10 - 29 | 10 |
| front_left_steer | motors | 10 - 29 | 11 |
| front_right_drive | motors | 10 - 29 | 12 |
| front_right_steer | motors | 10 - 29 | 13 |
| back_left_drive | motors | 10 - 29 | 14 |
| back_left_steer | motors | 10 - 29 | 15 |
| back_right_drive | motors | 10 - 29 | 16 |
| back_right_steer | motors | 10 - 29 | 17 |
| front_left_encoder | sensors | 40 - 59 | 40 |
| front_right_encoder | sensors | 40 - 59 | 41 |
| back_left_encoder | sensors | 40 - 59 | 42 |
| back_right_encoder | sensors | 40 - 59 | 43 |

#### Configure network devices
| Device | IP Address | Subnet Mask |
| - | - | - |
| OpenMesh radio | `10.17.57.1` | `???.???.???.???` |
| roboRIO | `10.17.57.2` | `255.255.255.000` |
| Driver Station (laptop) | `10.17.57.5` | `255.000.000.000` |

### Install robotpy
* **IMPORTANT: Perform ALL operations in a python virtualenv**
#### Create virtualenv (if not previously done)
Recommend placing the virtualenv in the `mentorbot` repo folder under `.venv` (to keep everything together) however the virtualenv is local to your system and should not be uploaded (ignored in `.gitignore`)
```bash
cd <path-to-mentorbot-repo>
py -3 -m venv ./.venv
```
#### Workflow
1. **Activate virtualenv**
   (Virtualenv activation may differ depending on your operating system and terminal)
   * Git Bash (Windows)
     ```bash
     source <path-to-mentorbot-repo>/.venv/Scripts/activate
     ```
1. **Install / update robotpy**
   (must have internet connection)
   ```bash
   python -m pip install -U robotpy
   ```
   (examples: `robotpy`, `robotpy[ctre,navx]`, `robotpy[all]`) (see: [robotpy on pypi](https://pypi.org/project/robotpy/))
1. **Download python for roboRIO**
   (must have internet connection)
   ```bash
   python -m robotpy_installer download-python
   ```
1. **Download robotpy modules for roboRIO**
   (must have internet connection)
   ```bash
   python -m robotpy_installer download robotpy
   ```
   (examples: `robotpy`, `robotpy[ctre,navx]`, `robotpy[all]`) (see: [robotpy on pypi](https://pypi.org/project/robotpy/))
1. **Install python on roboRIO**
   (must be connected to roboRIO)
   ```bash
   python -m robotpy_installer install-python
   ```
1. **Upload robotpy modules to roboRIO**
   (must be connected to roboRIO)
   ```bash
   python -m robotpy_installer install robotpy
   ```
   (examples: `robotpy`, `robotpy[ctre,navx]`, `robotpy[all]`) (see: [robotpy on pypi](https://pypi.org/project/robotpy/))
1. **Deploy robotpy program**
   * To robot
     (must be connected to roboRIO)
     ```bash
     python robot.py deploy
     ```
   * To simulator
     ```bash
     python robot.py sim
     ```
