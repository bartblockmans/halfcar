# Halfcar Downhill Simulation

A comprehensive simulation framework for longitudinal two-dimensional dynamics of a half-car/full-bicycle model with unilateral contact constraints, featuring customizable downhill courses and detailed animations.

![Halfcar Downhill Simulation](matlab/downhill_mtb.gif)

## Overview

This repository provides two implementations of a sophisticated halfcar dynamics simulation:

- **MATLAB** - Original implementation with extensive testing
- **Python** - Complete port with identical functionality

The simulation models a bicycle/motorcycle descending various downhill terrains with realistic physics including suspension dynamics, tire contact mechanics, and aerodynamic forces.

## Key Features

- **5-DOF Halfcar Model** - Longitudinal dynamics with pitch, heave, and wheel hop
- **Unilateral Contact Constraints** - Realistic tire-road interaction
- **Customizable Courses** - Build complex downhill tracks from modular segments
- **Advanced Physics** - Suspension damping, aerodynamic drag, rolling resistance
- **Real-time Animation** - Detailed visualization of vehicle dynamics
- **Power Pacing** - Realistic propulsive force modeling

## Course Generation

Create custom downhill courses by assembling predefined segments in `CourseLibrary.m` (MATLAB) or `course_library.py` (Python):

- **Bumpy sections** - Random terrain variations
- **Jumps and drops** - Airborne dynamics
- **Staircases** - Step-like obstacles  
- **Gap jumps** - Discontinuous terrain
- **Rock gardens** - High-frequency roughness
- **Root sections** - Natural trail features
- **Curves** - Banking and camber effects

## Quick Start

### MATLAB
```matlab
% Run the main simulation
Main_downhill

% Select course and parameters
[course, info] = SelectCourse('extreme');
```

### Python
```bash
# Install dependencies
pip install -r python/requirements.txt

# Run simulation
cd python
python main.py
```

## Repository Structure

```
halfcar/
├── matlab/              # MATLAB implementation
│   ├── Main_downhill.m     # Main simulation script
│   ├── ComputeAccelerations.m  # Physics engine
│   ├── ConstructCourse.m      # Course generation
│   ├── animate_results.m      # Animation system
│   └── ...                   # Supporting functions
└── python/              # Python implementation
    ├── main.py              # Main simulation script
    ├── params.py            # Model parameters
    ├── physics/             # Physics and dynamics
    ├── road/                # Course generation
    ├── viz/                 # Visualization
    ├── sim/                 # Integration
    └── post/                # Post-processing
```

## Physics Model

The simulation implements a 5-degree-of-freedom halfcar model:

- **Chassis dynamics** - X, Y translation and pitch rotation
- **Suspension systems** - Front and rear wheel hop
- **Contact mechanics** - Point and circle contact models
- **Force modeling** - Gravity, suspension, tire, aerodynamic, and propulsive forces

## Requirements

### MATLAB
- MATLAB R2019b or later
- Optimization Toolbox (for ODE integration)

### Python
- Python 3.8+
- NumPy, SciPy, Matplotlib
- See `python/requirements.txt` for complete list

## Development Status

- **MATLAB**: Fully tested and validated
- **Python**: Complete port with identical functionality
- **Documentation**: Comprehensive inline comments
- **Animation**: High-quality real-time visualization

## Contributing

This is a research/educational project. Feel free to:
- Report issues or bugs
- Suggest improvements
- Fork for your own modifications

## License

This project is open source. Please cite appropriately if used in academic work.

---

*Originally developed in MATLAB, then ported to Python for broader accessibility.*