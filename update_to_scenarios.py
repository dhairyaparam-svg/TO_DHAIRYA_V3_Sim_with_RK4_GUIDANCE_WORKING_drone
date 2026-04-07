#!/usr/bin/env python
"""Update smc_io_guidance.py to use external scenarios.py"""

with open('smc_io_guidance.py', 'r') as f:
    content = f.read()

# Replace the import section
old_import = """import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

try:
    import plotly.graph_objects as go
    import plotly.subplots as sp
    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False

try:
    import dash
    from dash import dcc, html, Input, Output
    DASH_AVAILABLE = True
except ImportError:
    DASH_AVAILABLE = False"""

new_import = """import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from scenarios import build_params as scenarios_build_params, get_all_scenarios, get_scenario

try:
    import plotly.graph_objects as go
    import plotly.subplots as sp
    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False

try:
    import dash
    from dash import dcc, html, Input, Output
    DASH_AVAILABLE = True
except ImportError:
    DASH_AVAILABLE = False"""

content = content.replace(old_import, new_import)

# Remove old LANDING_SCENARIOS dict and build_params
old_scenario_section = content[content.find('# =============================================================================\n# Landing scenario definitions'):content.find('# =============================================================================\n# Main simulation loop')]

new_scenario_section = """# =============================================================================
# Build Parameters (delegates to scenarios.py)
# =============================================================================

def build_params(ls_number=1):
    \"\"\"
    Build complete parameter dictionary for a landing scenario.
    
    Scenarios are defined in scenarios.py - modify that file to add more!
    \"\"\"
    return scenarios_build_params(ls_number)


"""

content = content.replace(old_scenario_section, new_scenario_section)

with open('smc_io_guidance.py', 'w') as f:
    f.write(content)

print("✓ Successfully updated smc_io_guidance.py")
print("✓ Now imports scenarios from scenarios.py")
print("✓ Removed hardcoded LANDING_SCENARIOS dict")
print("\nYou can now:")
print("  1. Add scenarios in scenarios.py")
print("  2. Run: python scenarios.py  (to see all scenarios)")
print("  3. Run: python smc_io_guidance.py --scenario 1 --mode interactive")
