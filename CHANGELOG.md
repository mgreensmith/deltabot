# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/) 
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

## 0.3.0 - 2016-11-21
### Added
- Added GPLv3 [LICENSE.txt](LICENSE.txt) and preamble in all source files.
- Added this change log.

### Changed
- Renamed `arduino/DeltaBot_Arduino.pde` → `deltabot_Arduino/deltabot_Arduino.ino`. ([@todocono](https://github.com/todocono))
- `deltabot_Arduino.ino`: Changed servo pin assignments to pins 8, 9, 10. ([@todocono](https://github.com/todocono))
- Renamed `processing/DeltaBot_GUI.pde` → `deltabot_Processing/deltabot_Processing.pde`. ([@todocono](https://github.com/todocono))
- `deltabot_Processing.pde`: Implement new ControlP5 object getter methods. ([@todocono](https://github.com/todocono))
- `deltabot_Processing.pde`: Serial port is now configured via a tuneable variable.
- Renamed `README` → [README.md](README.md), converted to Markdown and added content.

## 0.2.0 - 2012-05-16
### Added
- Initial public release of Deltabot.

[Unreleased]: https://github.com/mgreensmith/deltabot/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/mgreensmith/deltabot/compare/v0.2.0...v0.3.0
