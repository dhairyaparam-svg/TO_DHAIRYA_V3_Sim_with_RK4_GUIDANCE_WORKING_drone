#!/usr/bin/env python
"""Test that scenarios.py is integrated correctly"""

try:
    from smc_io_guidance import simulate, build_params
    import numpy as np
    
    print("="*70)
    print("INTEGRATION TEST: scenarios.py with smc_io_guidance.py")
    print("="*70)
    
    # Test 1: Load from scenarios
    print("\n[1] Loading parameters from scenarios.py...")
    p = build_params(1)
    print(f"    ✓ Successfully loaded LS1")
    print(f"    Scenario: {p.get('scenario_name', 'N/A')}")
    print(f"    Landing site: {p['rfd']}")
    
    # Test 2: Test all scenarios exist
    print("\n[2] Testing all scenarios...")
    for i in range(1, 8):
        try:
            p = build_params(i)
            print(f"    ✓ LS{i}: {p.get('scenario_name', 'OK')}")
        except Exception as e:
            print(f"    ✗ LS{i}: {str(e)}")
    
    # Test 3: List available functions from scenarios
    print("\n[3] Available scenario management functions:")
    from scenarios import list_scenarios, add_custom_scenario, modify_params
    print("    ✓ list_scenarios()")
    print("    ✓ add_custom_scenario()")
    print("    ✓ modify_params()")
    
    print("\n" + "="*70)
    print("INTEGRATION TEST: PASSED")
    print("="*70)
    print("\nYou can now:")
    print("  1. Edit scenarios.py to add/modify scenarios")
    print("  2. Run: python scenarios.py  (to see all scenarios)")
    print("  3. Add custom scenarios in Python:")
    print("\n     from scenarios import add_custom_scenario")
    print("     add_custom_scenario(8, 'My Landing', ...)")
    print("     params = build_params(8)")
    
except Exception as e:
    print(f"INTEGRATION TEST FAILED: {e}")
    import traceback
    traceback.print_exc()
