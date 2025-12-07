#!/usr/bin/env python3
"""
Diagnostic tool to test if target positions are reachable
and identify singularity issues
"""
import numpy as np
import sys
sys.path.append('/home/kiran_gunathilaka/development/ros/robotic_manipulator_simulation/src/gp7_robot')

from gp7_robot.gp7_kinematics import GP7Kinematics


def test_position(kinematics, pos, label, initial_guess=None):
    """Test if a position is reachable"""
    print(f"\n{'='*60}")
    print(f"Testing: {label}")
    print(f"Target: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
    print(f"Distance from base: {np.linalg.norm(pos):.3f} m")
    
    try:
        # Try IK
        joints = kinematics.inverse_kinematics_upright(pos, initial_guess)
        
        # Verify with FK
        T = kinematics.forward_kinematics(joints)
        achieved = T[:3, 3]
        error = np.linalg.norm(achieved - pos)
        
        # Check singularity
        is_singular, manipulability = kinematics.check_singularity(joints)
        
        print(f"\n✓ IK Solution found:")
        print(f"  Joints (deg): {[f'{np.degrees(j):6.1f}' for j in joints]}")
        print(f"  Achieved: [{achieved[0]:.3f}, {achieved[1]:.3f}, {achieved[2]:.3f}]")
        print(f"  Error: {error*1000:.2f} mm")
        print(f"  Manipulability: {manipulability:.4f}", end='')
        
        if is_singular:
            print(" ⚠ NEAR SINGULARITY!")
        else:
            print(" ✓ Good")
        
        # Check joint limits
        within_limits = True
        for i, (j, (lower, upper)) in enumerate(zip(joints, kinematics.joint_limits)):
            if j < lower or j > upper:
                print(f"  ✗ Joint {i+1} VIOLATES LIMITS: {np.degrees(j):.1f}° [{np.degrees(lower):.1f}, {np.degrees(upper):.1f}]")
                within_limits = False
        
        if within_limits:
            print(f"  ✓ All joints within limits")
        
        if error < 0.01 and not is_singular and within_limits:
            print(f"\n✓✓✓ POSITION IS REACHABLE AND SAFE ✓✓✓")
            return joints
        elif error < 0.01 and not is_singular:
            print(f"\n⚠ POSITION REACHABLE BUT HAS LIMIT VIOLATIONS")
            return joints
        elif error < 0.01:
            print(f"\n⚠ POSITION REACHABLE BUT NEAR SINGULARITY")
            return joints
        else:
            print(f"\n✗ POSITION HAS HIGH ERROR")
            return None
            
    except Exception as e:
        print(f"\n✗ IK FAILED: {e}")
        return None


def main():
    print("\n" + "="*60)
    print("GP7 ROBOT POSITION REACHABILITY DIAGNOSTIC")
    print("="*60)
    
    kinematics = GP7Kinematics()
    
    # Test home configuration
    print("\n### TESTING HOME CONFIGURATION ###")
    home_joints = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]
    T_home = kinematics.forward_kinematics(home_joints)
    home_pos = T_home[:3, 3]
    is_singular, manip = kinematics.check_singularity(home_joints)
    
    print(f"Home joints: {[f'{np.degrees(j):6.1f}°' for j in home_joints]}")
    print(f"Home position: [{home_pos[0]:.3f}, {home_pos[1]:.3f}, {home_pos[2]:.3f}]")
    print(f"Manipulability: {manip:.4f}", end='')
    if is_singular:
        print(" ⚠ SINGULARITY!")
    else:
        print(" ✓ Safe")
    
    # Test all target positions
    box_b_pos = np.array([0.70, 0.30, 0.025])
    box_a_top = np.array([0.30, 0.40, 0.225])
    clearance = 0.15
    grasp_offset = 0.02
    
    positions = [
        (box_b_pos + np.array([0, 0, clearance]), "Above Box B", home_joints),
        (box_b_pos + np.array([0, 0, 0.07]), "Grasp Box B", None),
        (box_a_top + np.array([0, 0, clearance]), "Above Box A", None),
        (box_a_top + np.array([0, 0, grasp_offset]), "Place on Box A", None),
    ]
    
    results = []
    last_good_joints = home_joints
    
    for pos, label, initial in positions:
        if initial is None:
            initial = last_good_joints
        
        result = test_position(kinematics, pos, label, initial)
        results.append((label, result is not None))
        
        if result is not None:
            last_good_joints = result
    
    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    
    for label, success in results:
        status = "✓ PASS" if success else "✗ FAIL"
        print(f"{status}: {label}")
    
    all_pass = all([r[1] for r in results])
    
    if all_pass:
        print("\n✓✓✓ ALL POSITIONS REACHABLE - READY TO EXECUTE ✓✓✓")
    else:
        print("\n✗✗✗ SOME POSITIONS UNREACHABLE - ADJUST PARAMETERS ✗✗✗")
        print("\nSuggestions:")
        print("- Move boxes closer to robot base")
        print("- Increase clearance height")
        print("- Adjust box positions in world file")
    
    print("="*60 + "\n")


if __name__ == "__main__":
    main()
