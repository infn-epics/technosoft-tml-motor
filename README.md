# IBEK Templates for Device Abstraction

This project provides a collection of templates designed to instantiate IBEK-supported devices and create models and abstractions for these devices. The templates simplify the process of defining and configuring EPICS Input/Output Controllers (IOCs) for various hardware devices.

## Overview

The templates in this repository are written in YAML and Jinja2, enabling the generation of configuration files for IOCs. These templates abstract the complexity of device configuration, allowing users to focus on high-level definitions and parameters.

## Features

- **Device Abstraction**: Simplifies the process of defining devices by providing reusable templates.
- **IBEK Integration**: Fully compatible with the IBEK framework for EPICS IOC generation.
- **Modular Design**: Each device type has its own template, making it easy to extend and customize.
- **Support for Multiple Devices**: Includes templates for various device types, such as cameras and chillers.

## Directory Structure

ibek-templates/
├── README.md
├── adcamera/
│   └── adcamera.yaml.j2
├── chiller/
│   └── chiller.yaml.j2

- `adcamera/`: Contains templates for configuring area detector cameras.
- `chiller/`: Contains templates for configuring chiller devices.
- `LICENSE`: Licensing information for the project.
- `README.md`: Documentation for the project.

## Prerequisites

- **IBEK Framework**: Ensure that the IBEK framework is installed and configured in your environment.
- **EPICS Base**: Required for running the generated IOC configurations.

