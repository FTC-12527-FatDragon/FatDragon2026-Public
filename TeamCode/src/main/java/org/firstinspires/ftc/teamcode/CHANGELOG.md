# Changelog

All notable changes to the FatDragon2026 project will be documented in this file.

## [Unreleased] - 2025-12-02 (Auto-Aim & Refactoring)

### Added
- **Gimbal Auto-Aim**: Implemented automatic gimbal aiming using robot odometry and field coordinates.
    - `AimCalculator`: Utility class to calculate servo position based on robot and target pose (includes 4:1 gear ratio logic).
    - `GimbalAutoAimCommand`: Command to continuously track the Blue Basket while held.
    - `GimbalManualControlCommand`: Command for manual fine-tuning of the gimbal.
    - `GimbalResetCommand`: Command to reset gimbal to center (IDLE) position.
- **Constants**: Centralized gimbal position constants in `GimbalConstants.java`.

### Changed
- **Refactoring**: Cleaned up `Solo.java` to strictly follow Command-Based architecture.
    - Removed manual logic from the `run()` loop.
    - Moved all controls to `initialize()` using `FunctionalButton` bindings.
- **Wheel Subsystem**: Replaced hardcoded servo values (0.5, 1) with named constants (`upwardServoLow`, `upwardServoHigh`).
- **Gimbal Subsystem**:
    - Changed default initial position to 0.5 (Center).
    - Updated `periodic()` logic to support both State Machine and direct Manual Control.

### Fixed
- Fixed issue where gimbal manual control logic was cluttering the OpMode loop.
- Fixed hardcoded values in `Wheel.java` toggle method.

### Usage
- **D-Pad Down (Hold)**: Activate Auto-Aim (tracks Blue Basket).
- **D-Pad Up (Press)**: Reset Gimbal to Center.
- **D-Pad Left/Right (Hold)**: Manual Gimbal Fine-Tuning.

