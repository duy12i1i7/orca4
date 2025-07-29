#!/usr/bin/env python3

"""
Multi-AUV Formation Launch Helper

Script tự động khởi chạy formation controller phù hợp và mission runner.
Hỗ trợ các thuật toán khác nhau và tự động điều phối.
"""

import subprocess
import time
import sys
import os
import signal
from enum import Enum


class FormationMode(Enum):
    BASIC = "basic"                    # formation_controller.py (đơn giản)
    MADDPG_RBF = "maddpg_rbf"         # maddpg_rbf_controller.py (đầy đủ)
    LEADER_CONTROLLED = "leader"       # leader_trajectory_controller.py


class MissionType(Enum):
    BASIC_MANEUVERS = "basic"
    WAYPOINT_TRAJECTORY = "waypoint"   # Theo mophong.txt
    SINE_WAVE = "sine"                 # Quỹ đạo hình sin


class FormationLauncher:
    def __init__(self):
        self.processes = []
        self.formation_mode = FormationMode.BASIC
        self.mission_type = MissionType.BASIC_MANEUVERS
        
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        print("\n\n🛑 Stopping all processes...")
        self.cleanup()
        sys.exit(0)
        
    def cleanup(self):
        """Terminate all spawned processes"""
        for process in self.processes:
            if process.poll() is None:  # Process is still running
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
        self.processes.clear()
        
    def run_command(self, command, description, background=True):
        """Run a ROS2 command"""
        print(f"🚀 Starting: {description}")
        print(f"   Command: {command}")
        
        if background:
            process = subprocess.Popen(command, shell=True)
            self.processes.append(process)
            time.sleep(2)  # Give time to start
            return process
        else:
            return subprocess.run(command, shell=True)
    
    def select_formation_mode(self):
        """Interactive selection of formation control mode"""
        print("\n" + "="*60)
        print("  🤖 FORMATION CONTROL MODE SELECTION")
        print("="*60)
        print("1. Basic Formation Controller (Recommended)")
        print("   - Simplified MADDPG+RBF implementation")
        print("   - Good for testing and demonstrations")
        print("")
        print("2. Full MADDPG+RBF Controller")
        print("   - Complete RBF network simulation")
        print("   - Advanced multi-agent learning")
        print("")
        print("3. Leader Trajectory Controller")
        print("   - Predefined leader trajectories")
        print("   - Sine wave or waypoint following")
        print("="*60)
        
        while True:
            try:
                choice = input("Select mode (1-3, default=1): ").strip()
                if not choice:
                    choice = "1"
                    
                if choice == "1":
                    self.formation_mode = FormationMode.BASIC
                    break
                elif choice == "2":
                    self.formation_mode = FormationMode.MADDPG_RBF
                    break
                elif choice == "3":
                    self.formation_mode = FormationMode.LEADER_CONTROLLED
                    break
                else:
                    print("❌ Invalid choice. Please select 1, 2, or 3.")
            except KeyboardInterrupt:
                print("\n\n👋 Goodbye!")
                sys.exit(0)
    
    def select_mission_type(self):
        """Interactive selection of mission type"""
        print("\n" + "="*60) 
        print("  🎯 MISSION TYPE SELECTION")
        print("="*60)
        print("1. Basic Maneuvers (Default)")
        print("   - Simple forward, turn, circle movements")
        print("   - Good for testing formation control")
        print("")
        print("2. Waypoint Trajectory (from mophong.txt)")
        print("   - (0,0) → (20,-13) → (10,-23) → (-10,-8) → (0,0)")
        print("   - 2 cycles, ≈2 m/s speed")
        print("")
        print("3. Sine Wave Trajectory") 
        print("   - x₁(t) = v*t, y₁(t) = A*sin(ω*t)")
        print("   - Smooth oscillating movement")
        print("="*60)
        
        while True:
            try:
                choice = input("Select mission (1-3, default=1): ").strip()
                if not choice:
                    choice = "1"
                    
                if choice == "1":
                    self.mission_type = MissionType.BASIC_MANEUVERS
                    break
                elif choice == "2":
                    self.mission_type = MissionType.WAYPOINT_TRAJECTORY
                    break
                elif choice == "3":
                    self.mission_type = MissionType.SINE_WAVE
                    break
                else:
                    print("❌ Invalid choice. Please select 1, 2, or 3.")
            except KeyboardInterrupt:
                print("\n\n👋 Goodbye!")
                sys.exit(0)
    
    def launch_formation_controller(self):
        """Launch the selected formation controller"""
        controllers = {
            FormationMode.BASIC: ("ros2 run orca_base formation_controller.py", 
                                 "Basic Formation Controller (MADDPG+RBF Simplified)"),
            FormationMode.MADDPG_RBF: ("ros2 run orca_base maddpg_rbf_controller.py",
                                      "Full MADDPG+RBF Controller"),
            FormationMode.LEADER_CONTROLLED: ("ros2 run orca_base leader_trajectory_controller.py",
                                             "Leader Trajectory Controller")
        }
        
        command, description = controllers[self.formation_mode]
        return self.run_command(command, description)
        
    def launch_mission_runner(self):
        """Launch mission runner with appropriate configuration"""
        if self.formation_mode == FormationMode.LEADER_CONTROLLED:
            print("ℹ️  Leader Trajectory Controller handles movement automatically")
            print("   No additional mission runner needed.")
            return None
        
        mission_commands = {
            MissionType.BASIC_MANEUVERS: "ros2 run orca_base formation_mission_runner.py",
            MissionType.WAYPOINT_TRAJECTORY: "ros2 run orca_base formation_mission_runner.py", 
            MissionType.SINE_WAVE: "ros2 run orca_base formation_mission_runner.py"
        }
        
        command = mission_commands[self.mission_type]
        description = f"Mission Runner ({self.mission_type.value})"
        
        print(f"\nℹ️  Note: Modify formation_mission_runner.py main() to select:")
        if self.mission_type == MissionType.WAYPOINT_TRAJECTORY:
            print("   Uncomment: mission_runner.maddpg_formation_mission()")
        elif self.mission_type == MissionType.SINE_WAVE:
            print("   Uncomment: mission_runner.sine_wave_trajectory(60.0)")
            
        return self.run_command(command, description, background=False)
    
    def display_summary(self):
        """Display launch configuration summary"""
        print("\n" + "="*60)
        print("  🚀 LAUNCH CONFIGURATION")
        print("="*60)
        print(f"Formation Mode: {self.formation_mode.value}")
        print(f"Mission Type:  {self.mission_type.value}")
        print("")
        print("Expected behavior:")
        print("- AUV1 (Leader): Executes mission commands")
        print("- AUV2 (Follower): Δ₁₂ = (-5, 2) m [behind+left]")
        print("- AUV3 (Follower): Δ₁₃ = (-5, -2) m [behind+right]")
        print("")
        print("Formation control frequency: 20Hz")
        print("Formation error tolerance: Minimized by MADDPG+RBF")
        print("="*60)
        
    def run(self):
        """Main execution flow"""
        # Setup signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        
        print("🌊 Multi-AUV Formation Control with MADDPG+RBF")
        print("   Based on algorithm described in mophong.txt")
        
        # Interactive selection
        self.select_formation_mode()
        self.select_mission_type()
        self.display_summary()
        
        input("\n⏯️  Press Enter to start the formation control system...")
        
        try:
            # Launch formation controller
            controller_process = self.launch_formation_controller()
            
            print(f"\n⏳ Waiting 5 seconds for formation controller to initialize...")
            time.sleep(100)
            
            # Launch mission runner
            mission_process = self.launch_mission_runner()
            
            print("\n✅ Formation system is running!")
            print("   Monitor the AUVs in Gazebo and RViz")
            print("   Press Ctrl+C to stop all processes")
            
            # Keep running until interrupted
            if mission_process is None:
                # Leader controlled mode - just wait
                print("\n🔄 Leader Trajectory Controller is running...")
                print("   The leader will follow predefined trajectories")
                while True:
                    time.sleep(1)
            else:
                # Wait for mission to complete
                mission_process.wait()
                
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()
            print("\n✅ All processes stopped. Formation mission complete!")


def main():
    launcher = FormationLauncher()
    launcher.run()


if __name__ == "__main__":
    main()
