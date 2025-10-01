def course_library():
    """Return a dict of predefined courses (port of CourseLibrary.m)."""
    courses = {}
    # Basic test course
    courses["basic_test"] = {
        "name": "Basic Test Course",
        "description": "Simple course with flat sections and one jump",
        "segments": [
            {"type":"flat","distance":20,"gradient":0},
            {"type":"jump","distance":10,"gradient":-0.05,"height":0.5},
            {"type":"flat","distance":30,"gradient":0},
        ],
    }
    # Complex downhill course
    courses["complex_downhill"] = {
        "name": "Complex Downhill",
        "description": "Challenging course with bumps, jumps, and drops",
        "segments": [
            {"type":"flat","distance":30,"gradient":0},
            {"type":"bumpy","distance":20,"gradient":-0.05,"height":0.03,"frequency":3},
            {"type":"jump","distance":15,"gradient":-0.1,"height":1.5},
            {"type":"bumpy","distance":25,"gradient":-0.15,"height":0.08,"frequency":1.5},
            {"type":"drop","distance":12,"gradient":-0.3,"height":1.2},
            {"type":"jump","distance":18,"gradient":-0.08,"height":0.8},
            {"type":"flat","distance":40,"gradient":0},
        ],
    }
    # Extreme course
    courses["extreme"] = {
        "name": "Extreme Course",
        "description": "High-speed course with large features",
        "segments": [
            {"type":"flat","distance":20,"gradient":-0.1},
            {"type":"jump","distance":20,"gradient":-0.2,"height":0.5},
            {"type":"drop","distance":15,"gradient":-0.4,"height":0.5},
            {"type":"bumpy","distance":30,"gradient":-0.15,"height":0.10,"frequency":2},
            {"type":"jump","distance":40,"gradient":-0.25,"height":0.5},
            {"type":"staircase","distance":50,"gradient":0,"height":4.0,"step_length":0.5,"num_steps":16},
            {"type":"jump","distance":30,"gradient":-0.2,"height":1.0},
            {"type":"flat","distance":20,"gradient":0},
            {"type":"trapezium","distance":20,"gradient":-0.1,"height":1.2,"flat_length":10.0,"ramp_inclination":15},
            {"type":"flat","distance":20,"gradient":0},
            {"type":"rock_garden","distance":30,"gradient":-0.15,"height":0.2,"rock_density":5},
            {"type":"gap_jump","distance":15,"gradient":-0.1,"height":1.0,"gap_width":8.0},
            {"type":"root_section","distance":30,"gradient":-0.1,"height":0.1,"root_density":8},
            {"type":"flat","distance":20,"gradient":0},
        ],
    }
    # Technical course
    courses["technical"] = {
        "name": "Technical Course",
        "description": "Course with many small features requiring precision",
        "segments": [
            {"type":"flat","distance":15,"gradient":0},
            {"type":"bumpy","distance":10,"gradient":-0.02,"height":0.02,"frequency":5},
            {"type":"jump","distance":8,"gradient":-0.03,"height":0.3},
            {"type":"bumpy","distance":12,"gradient":-0.05,"height":0.04,"frequency":4},
            {"type":"drop","distance":6,"gradient":-0.08,"height":0.4},
            {"type":"bumpy","distance":15,"gradient":-0.06,"height":0.03,"frequency":6},
            {"type":"jump","distance":10,"gradient":-0.04,"height":0.5},
            {"type":"flat","distance":25,"gradient":0},
        ],
    }
    # Speed course
    courses["speed"] = {
        "name": "Speed Course",
        "description": "Long downhill course optimized for high speed",
        "segments": [
            {"type":"flat","distance":100,"gradient":-0.05},
            {"type":"jump","distance":30,"gradient":-0.08,"height":1.0},
            {"type":"flat","distance":80,"gradient":-0.12},
            {"type":"jump","distance":25,"gradient":-0.15,"height":1.5},
            {"type":"flat","distance":120,"gradient":-0.1},
            {"type":"drop","distance":20,"gradient":-0.2,"height":1.8},
            {"type":"flat","distance":150,"gradient":0},
        ],
    }
    # Beginner course
    courses["beginner"] = {
        "name": "Beginner Course",
        "description": "Easy course with gentle features",
        "segments": [
            {"type":"flat","distance":40,"gradient":0},
            {"type":"bumpy","distance":20,"gradient":-0.02,"height":0.01,"frequency":2},
            {"type":"jump","distance":12,"gradient":-0.03,"height":0.2},
            {"type":"flat","distance":30,"gradient":-0.01},
            {"type":"drop","distance":8,"gradient":-0.05,"height":0.3},
            {"type":"flat","distance":50,"gradient":0},
        ],
    }
    return courses
