#!/usr/bin/env python3
"""
Trajectory Interpolation Log Visualizer - Documentation Style with Effort

Creates plots similar to ROS2 control documentation showing: 
- Position vs Time with waypoints
- Velocity vs Time with specified velocity waypoints
- Effort vs Time with specified effort waypoints

Usage:
    python3 visualize_trajectory_log. py [log_file]
    python3 visualize_trajectory_log.py log.txt --style doc --joint 0
    python3 visualize_trajectory_log.py log.txt --style all --save output
"""

import sys
import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import argparse


@dataclass
class TrajectoryPoint:
    time: float
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    efforts: List[float] = field(default_factory=list)


@dataclass
class SampledPoint:
    time: float
    pos: List[float] = field(default_factory=list)
    vel: List[float] = field(default_factory=list)
    eff: List[float] = field(default_factory=list)
    cmd_pos: List[float] = field(default_factory=list)
    cmd_vel: List[float] = field(default_factory=list)
    cmd_eff: List[float] = field(default_factory=list)
    fb_pos: List[float] = field(default_factory=list)
    err_pos: List[float] = field(default_factory=list)


@dataclass
class TrajectoryLog:
    config_name:  str = ""
    interpolation_method: str = "splines"
    num_joints: int = 0
    joint_names: List[str] = field(default_factory=list)
    input_points: List[TrajectoryPoint] = field(default_factory=list)
    sampled_points: List[SampledPoint] = field(default_factory=list)


def clean_line(line: str) -> str:
    """Remove CTest prefix (e.g., '3:  ') from line."""
    cleaned = re.sub(r'^\d+:\s*', '', line)
    return cleaned. strip()


def parse_float_list(s: str) -> List[float]:
    """Parse comma-separated floats from string."""
    s = s.strip()
    if not s:
        return []
    try:
        return [float(x. strip()) for x in s.split(',') if x.strip()]
    except ValueError:
        return []


def parse_log(log_text: str) -> List[TrajectoryLog]:
    """Parse the trajectory log text and return list of TrajectoryLog objects."""
    logs = []
    current_log = None
    current_input_point = None
    current_sample = None
    in_input_section = False
    in_sampled_section = False

    lines = log_text.strip().split('\n')
    
    for raw_line in lines:
        line = clean_line(raw_line)
        
        if not line:
            continue
        
        # New log section
        if line.startswith("CONFIG:"):
            if current_log is not None:
                logs.append(current_log)
            current_log = TrajectoryLog()
            current_log.config_name = line.replace("CONFIG:", "").strip()
            in_input_section = False
            in_sampled_section = False
            
        elif line.startswith("INTERPOLATION:"):
            if current_log: 
                current_log.interpolation_method = line.replace("INTERPOLATION:", "").strip()
            
        elif line == "--- INPUT_TRAJECTORY_START ---":
            in_input_section = True
            in_sampled_section = False
            
        elif line == "--- INPUT_TRAJECTORY_END ---":
            in_input_section = False
            if current_input_point is not None:
                current_log.input_points.append(current_input_point)
                current_input_point = None
                
        elif line == "--- SAMPLED_TRAJECTORY_START ---":
            in_sampled_section = True
            in_input_section = False
            
        elif line == "--- SAMPLED_TRAJECTORY_END ---":
            in_sampled_section = False
            if current_sample is not None:
                current_log.sampled_points. append(current_sample)
                current_sample = None
                
        elif current_log is not None: 
            # Parse input trajectory section
            if in_input_section:
                if line.startswith("NUM_JOINTS:"):
                    current_log.num_joints = int(line.replace("NUM_JOINTS:", "").strip())
                elif line.startswith("JOINT_NAMES:"):
                    names = line.replace("JOINT_NAMES:", "").strip()
                    current_log.joint_names = [n.strip() for n in names.split(',')]
                elif line.startswith("POINT "):
                    if current_input_point is not None:
                        current_log. input_points.append(current_input_point)
                    current_input_point = TrajectoryPoint(time=0.0)
                elif line.startswith("TIME:") and current_input_point is not None:
                    current_input_point.time = float(line.replace("TIME:", "").strip())
                elif line.startswith("POSITIONS:") and current_input_point is not None:
                    current_input_point.positions = parse_float_list(line.replace("POSITIONS:", ""))
                elif line.startswith("VELOCITIES:") and current_input_point is not None: 
                    current_input_point. velocities = parse_float_list(line.replace("VELOCITIES:", ""))
                elif line. startswith("EFFORTS:") and current_input_point is not None:
                    current_input_point.efforts = parse_float_list(line.replace("EFFORTS:", ""))
                        
            # Parse sampled trajectory section
            elif in_sampled_section:
                if line.startswith("SAMPLE "):
                    if current_sample is not None: 
                        current_log.sampled_points.append(current_sample)
                    current_sample = SampledPoint(time=0.0)
                elif line.startswith("TIME:") and current_sample is not None: 
                    current_sample.time = float(line.replace("TIME:", "").strip())
                # New format
                elif line.startswith("POS: ") and current_sample is not None:
                    current_sample. pos = parse_float_list(line.replace("POS:", ""))
                elif line.startswith("VEL:") and current_sample is not None:
                    current_sample.vel = parse_float_list(line.replace("VEL:", ""))
                elif line.startswith("EFF:") and current_sample is not None:
                    current_sample.eff = parse_float_list(line.replace("EFF:", ""))
                # Old format (backwards compatibility)
                elif line.startswith("REF_POS:") and current_sample is not None:
                    current_sample.pos = parse_float_list(line.replace("REF_POS:", ""))
                elif line. startswith("REF_VEL:") and current_sample is not None:
                    current_sample.vel = parse_float_list(line.replace("REF_VEL:", ""))
                elif line.startswith("REF_EFF:") and current_sample is not None:
                    current_sample.eff = parse_float_list(line. replace("REF_EFF:", ""))
                # Command values
                elif line.startswith("CMD_POS:") and current_sample is not None:
                    current_sample.cmd_pos = parse_float_list(line.replace("CMD_POS:", ""))
                elif line.startswith("CMD_VEL: ") and current_sample is not None:
                    current_sample. cmd_vel = parse_float_list(line.replace("CMD_VEL:", ""))
                elif line.startswith("CMD_EFF:") and current_sample is not None:
                    current_sample.cmd_eff = parse_float_list(line. replace("CMD_EFF:", ""))
                # Feedback and error
                elif line.startswith("FB_POS:") and current_sample is not None:
                    current_sample.fb_pos = parse_float_list(line.replace("FB_POS:", ""))
                elif line.startswith("ERR_POS:") and current_sample is not None:
                    current_sample.err_pos = parse_float_list(line.replace("ERR_POS:", ""))

    # Don't forget the last log
    if current_log is not None: 
        if current_input_point is not None:
            current_log.input_points.append(current_input_point)
        if current_sample is not None:
            current_log.sampled_points. append(current_sample)
        logs.append(current_log)

    return logs


def extract_data(log: TrajectoryLog) -> Optional[dict]:
    """Extract numpy arrays from log data."""
    if not log.sampled_points:
        return None
    
    num_joints = len(log.sampled_points[0].pos) if log.sampled_points[0].pos else 3
    n_samples = len(log.sampled_points)
    n_inputs = len(log.input_points)
    
    # Sample data arrays
    data = {
        'times': np.array([s.time for s in log.sampled_points]),
        'pos': np.zeros((n_samples, num_joints)),
        'vel': np. zeros((n_samples, num_joints)),
        'eff': np.zeros((n_samples, num_joints)),
        'cmd_pos': np.zeros((n_samples, num_joints)),
        'cmd_vel': np. zeros((n_samples, num_joints)),
        'cmd_eff': np.zeros((n_samples, num_joints)),
        'fb_pos': np.zeros((n_samples, num_joints)),
        'err_pos':  np.zeros((n_samples, num_joints)),
        'input_times': np.array([p.time for p in log.input_points]) if n_inputs > 0 else np.array([]),
        'input_pos': np.zeros((n_inputs, num_joints)),
        'input_vel': np.zeros((n_inputs, num_joints)),
        'input_eff':  np.zeros((n_inputs, num_joints)),
        'has_input_vel': False,
        'has_input_eff': False,
        'num_joints': num_joints,
    }
    
    # Fill sample data
    for i, s in enumerate(log.sampled_points):
        if s.pos: 
            data['pos'][i, :len(s.pos)] = s.pos
        if s.vel:
            data['vel'][i, :len(s.vel)] = s.vel
        if s. eff:
            data['eff'][i, :len(s.eff)] = s.eff
        if s.cmd_pos:
            data['cmd_pos'][i, :len(s.cmd_pos)] = s.cmd_pos
        if s. cmd_vel:
            data['cmd_vel'][i, :len(s.cmd_vel)] = s.cmd_vel
        if s.cmd_eff:
            data['cmd_eff'][i, :len(s.cmd_eff)] = s.cmd_eff
        if s. fb_pos:
            data['fb_pos'][i, :len(s.fb_pos)] = s.fb_pos
        if s.err_pos:
            data['err_pos'][i, :len(s.err_pos)] = s.err_pos
    
    # Fill input waypoint data
    for i, p in enumerate(log. input_points):
        if p.positions:
            data['input_pos'][i, :len(p. positions)] = p.positions
        if p.velocities:
            data['input_vel'][i, :len(p.velocities)] = p.velocities
            data['has_input_vel'] = True
        if p.efforts:
            data['input_eff'][i, :len(p. efforts)] = p.efforts
            data['has_input_eff'] = True
    
    return data


def plot_documentation_style(log: TrajectoryLog, joint_idx: int = 0, save_path: Optional[str] = None):
    """
    Create documentation-style plot for a single joint.
    Shows Position, Velocity, and Effort stacked vertically.
    Includes waypoint markers for position, velocity, and effort.
    """
    data = extract_data(log)
    if data is None:
        print(f"No data for {log.config_name}")
        return None
    
    joint_name = log.joint_names[joint_idx] if joint_idx < len(log.joint_names) else f"Joint {joint_idx + 1}"
    
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f'{log.config_name}\n{log.interpolation_method. upper()} Interpolation - {joint_name}', 
                 fontsize=12, fontweight='bold')
    
    color_interp = '#1f77b4'      # Blue for interpolated line
    color_waypoint = '#d62728'    # Red for position waypoints
    color_vel_wp = '#2ca02c'      # Green for velocity waypoints
    color_eff_wp = '#9467bd'      # Purple for effort waypoints
    
    times = data['times']
    input_times = data['input_times']
    
    # =========== Position plot ===========
    ax1 = axes[0]
    ax1.plot(times, data['pos'][:, joint_idx], '-', color=color_interp, 
             linewidth=2, label='Interpolated')
    if len(input_times) > 0:
        ax1.scatter(input_times, data['input_pos'][:, joint_idx], color=color_waypoint, s=100, 
                    marker='o', edgecolors='black', linewidths=1.5, zorder=5, label='Waypoints')
    ax1.set_ylabel('Position [rad]', fontsize=10)
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Position', fontsize=10, loc='left')
    
    # =========== Velocity plot ===========
    ax2 = axes[1]
    ax2.plot(times, data['vel'][:, joint_idx], '-', color=color_interp, 
             linewidth=2, label='Interpolated')
    
    # Show specified velocity waypoints if they exist
    if data['has_input_vel'] and len(input_times) > 0:
        ax2.scatter(input_times, data['input_vel'][:, joint_idx], color=color_vel_wp, s=100, 
                    marker='^', edgecolors='black', linewidths=1.5, zorder=5, label='Specified')
    
    ax2.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    ax2.set_ylabel('Velocity [rad/s]', fontsize=10)
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Velocity', fontsize=10, loc='left')
    
    # =========== Effort plot ===========
    ax3 = axes[2]
    ax3.plot(times, data['eff'][:, joint_idx], '-', color=color_interp, 
             linewidth=2, label='Interpolated')
    
    # Show specified effort waypoints if they exist
    if data['has_input_eff'] and len(input_times) > 0:
        ax3.scatter(input_times, data['input_eff'][:, joint_idx], color=color_eff_wp, s=100, 
                    marker='s', edgecolors='black', linewidths=1.5, zorder=5, label='Specified')
    
    # Optionally show command effort if different from reference
    cmd_eff = data['cmd_eff'][:, joint_idx]
    ref_eff = data['eff'][:, joint_idx]
    if np.any(cmd_eff != 0) and not np.allclose(cmd_eff, ref_eff, atol=0.01):
        ax3.plot(times, cmd_eff, '--', color='#ff7f0e', linewidth=1.5, alpha=0.7, label='Command')
    
    ax3.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    ax3.set_ylabel('Effort [Nm]', fontsize=10)
    ax3.set_xlabel('Time [s]', fontsize=10)
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    ax3.set_title('Effort', fontsize=10, loc='left')
    
    # Add vertical lines at waypoint times
    for t in input_times:
        for ax in axes:
            ax.axvline(x=t, color='gray', linestyle=':', alpha=0.5)
    
    plt.tight_layout()
    
    if save_path:
        plt. savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved:  {save_path}")
    
    return fig


def plot_all_joints(log: TrajectoryLog, save_path: Optional[str] = None):
    """
    Create plot showing all joints on same axes.
    """
    data = extract_data(log)
    if data is None:
        print(f"No data for {log.config_name}")
        return None
    
    num_joints = data['num_joints']
    times = data['times']
    input_times = data['input_times']
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle(f'{log.config_name} - All Joints\n{log.interpolation_method.upper()} Interpolation', 
                 fontsize=12, fontweight='bold')
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    joint_labels = log.joint_names if log.joint_names else [f'Joint {i+1}' for i in range(num_joints)]
    
    # Position
    ax1 = axes[0]
    for j in range(num_joints):
        ax1.plot(times, data['pos'][:, j], '-', color=colors[j % len(colors)], 
                 linewidth=2, label=joint_labels[j])
        if len(input_times) > 0:
            ax1.scatter(input_times, data['input_pos'][:, j], color=colors[j % len(colors)], 
                        s=80, marker='o', edgecolors='black', linewidths=1, zorder=5)
    ax1.set_ylabel('Position [rad]')
    ax1.legend(loc='upper right', ncol=min(num_joints, 3))
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Position (○ = waypoints)', loc='left')
    
    # Velocity
    ax2 = axes[1]
    for j in range(num_joints):
        ax2.plot(times, data['vel'][:, j], '-', color=colors[j % len(colors)], 
                 linewidth=2, label=joint_labels[j])
        if data['has_input_vel'] and len(input_times) > 0:
            ax2.scatter(input_times, data['input_vel'][:, j], color=colors[j % len(colors)], 
                        s=80, marker='^', edgecolors='black', linewidths=1, zorder=5)
    ax2.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    ax2.set_ylabel('Velocity [rad/s]')
    ax2.legend(loc='upper right', ncol=min(num_joints, 3))
    ax2.grid(True, alpha=0.3)
    title_vel = 'Velocity (△ = specified)' if data['has_input_vel'] else 'Velocity'
    ax2.set_title(title_vel, loc='left')
    
    # Effort
    ax3 = axes[2]
    for j in range(num_joints):
        ax3.plot(times, data['eff'][:, j], '-', color=colors[j % len(colors)], 
                 linewidth=2, label=joint_labels[j])
        if data['has_input_eff'] and len(input_times) > 0:
            ax3.scatter(input_times, data['input_eff'][:, j], color=colors[j % len(colors)], 
                        s=80, marker='s', edgecolors='black', linewidths=1, zorder=5)
    ax3.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    ax3.set_ylabel('Effort [Nm]')
    ax3.set_xlabel('Time [s]')
    ax3.legend(loc='upper right', ncol=min(num_joints, 3))
    ax3.grid(True, alpha=0.3)
    title_eff = 'Effort (□ = specified)' if data['has_input_eff'] else 'Effort'
    ax3.set_title(title_eff, loc='left')
    
    # Waypoint time markers
    for t in input_times:
        for ax in axes:
            ax. axvline(x=t, color='gray', linestyle=':', alpha=0.4)
    
    plt.tight_layout()
    
    if save_path:
        plt. savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_comparison(logs: List[TrajectoryLog], joint_idx: int = 0, save_path: Optional[str] = None):
    """
    Compare multiple trajectory configurations on same plot.
    """
    if len(logs) < 2:
        print("Need at least 2 logs for comparison")
        return None
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle(f'Interpolation Comparison - Joint {joint_idx + 1}', 
                 fontsize=12, fontweight='bold')
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    
    first_data = None
    for idx, log in enumerate(logs):
        data = extract_data(log)
        if data is None: 
            continue
        
        if first_data is None:
            first_data = data
        
        color = colors[idx % len(colors)]
        label = f"{log.config_name} ({log.interpolation_method})"
        
        # Position
        axes[0].plot(data['times'], data['pos'][:, joint_idx], '-', color=color, 
                     linewidth=2, label=label)
        
        # Velocity
        axes[1].plot(data['times'], data['vel'][:, joint_idx], '-', color=color, 
                     linewidth=2, label=label)
        
        # Effort
        axes[2].plot(data['times'], data['eff'][:, joint_idx], '-', color=color, 
                     linewidth=2, label=label)
    
    # Show waypoints from first trajectory
    if first_data is not None and len(first_data['input_times']) > 0:
        axes[0].scatter(first_data['input_times'], first_data['input_pos'][:, joint_idx], 
                       color='black', s=100, marker='o', zorder=5, label='Waypoints')
        if first_data['has_input_vel']: 
            axes[1].scatter(first_data['input_times'], first_data['input_vel'][:, joint_idx], 
                           color='black', s=80, marker='^', zorder=5, label='Specified')
        if first_data['has_input_eff']:
            axes[2].scatter(first_data['input_times'], first_data['input_eff'][:, joint_idx], 
                           color='black', s=80, marker='s', zorder=5, label='Specified')
    
    axes[0].set_ylabel('Position [rad]')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Position', loc='left')
    
    axes[1].set_ylabel('Velocity [rad/s]')
    axes[1].axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('Velocity', loc='left')
    
    axes[2].set_ylabel('Effort [Nm]')
    axes[2].set_xlabel('Time [s]')
    axes[2].axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title('Effort', loc='left')
    
    plt.tight_layout()
    
    if save_path: 
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved:  {save_path}")
    
    return fig


def plot_phase_portrait(log: TrajectoryLog, save_path: Optional[str] = None):
    """
    Create phase portrait (position vs velocity) plot.
    """
    data = extract_data(log)
    if data is None:
        return None
    
    num_joints = data['num_joints']
    
    fig, axes = plt.subplots(1, num_joints, figsize=(4*num_joints, 4))
    if num_joints == 1:
        axes = [axes]
    
    fig.suptitle(f'{log.config_name} - Phase Portraits', fontsize=12, fontweight='bold')
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']
    joint_labels = log.joint_names if log.joint_names else [f'Joint {i+1}' for i in range(num_joints)]
    
    for j, ax in enumerate(axes):
        pos = data['pos'][:, j]
        vel = data['vel'][:, j]
        
        ax.plot(pos, vel, '-', color=colors[j % len(colors)], linewidth=2)
        ax.scatter(pos[0], vel[0], color='green', s=100, marker='o', 
                   edgecolors='black', zorder=5, label='Start')
        ax.scatter(pos[-1], vel[-1], color='red', s=100, marker='s', 
                   edgecolors='black', zorder=5, label='End')
        
        # Mark waypoints
        if len(data['input_times']) > 0:
            for i in range(len(data['input_pos'])):
                wp_pos = data['input_pos'][i, j]
                wp_vel = data['input_vel'][i, j] if data['has_input_vel'] else 0
                ax.scatter(wp_pos, wp_vel, color='black', s=50, marker='x', zorder=4)
        
        ax.set_xlabel('Position [rad]')
        ax.set_ylabel('Velocity [rad/s]')
        ax.set_title(joint_labels[j])
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
        ax.axvline(x=0, color='gray', linestyle='--', linewidth=0.5)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_command_vs_reference(log: TrajectoryLog, joint_idx: int = 0, save_path: Optional[str] = None):
    """
    Plot command values vs reference values to see controller behavior.
    """
    data = extract_data(log)
    if data is None:
        return None
    
    joint_name = log.joint_names[joint_idx] if joint_idx < len(log.joint_names) else f"Joint {joint_idx + 1}"
    
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f'{log.config_name} - Reference vs Command\n{joint_name}', 
                 fontsize=12, fontweight='bold')
    
    times = data['times']
    input_times = data['input_times']
    
    # Position
    ax1 = axes[0]
    ax1.plot(times, data['pos'][:, joint_idx], '-', color='#1f77b4', linewidth=2, label='Reference')
    ax1.plot(times, data['cmd_pos'][:, joint_idx], '--', color='#ff7f0e', linewidth=1.5, label='Command')
    if len(input_times) > 0:
        ax1.scatter(input_times, data['input_pos'][:, joint_idx], color='black', s=80, 
                    marker='o', zorder=5, label='Waypoints')
    ax1.set_ylabel('Position [rad]')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Position', loc='left')
    
    # Velocity
    ax2 = axes[1]
    ax2.plot(times, data['vel'][:, joint_idx], '-', color='#1f77b4', linewidth=2, label='Reference')
    ax2.plot(times, data['cmd_vel'][:, joint_idx], '--', color='#ff7f0e', linewidth=1.5, label='Command')
    if data['has_input_vel'] and len(input_times) > 0:
        ax2.scatter(input_times, data['input_vel'][:, joint_idx], color='black', s=80, 
                    marker='^', zorder=5, label='Specified')
    ax2.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    ax2.set_ylabel('Velocity [rad/s]')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Velocity', loc='left')
    
    # Effort
    ax3 = axes[2]
    ax3.plot(times, data['eff'][:, joint_idx], '-', color='#1f77b4', linewidth=2, label='Reference')
    ax3.plot(times, data['cmd_eff'][:, joint_idx], '--', color='#ff7f0e', linewidth=1.5, label='Command')
    if data['has_input_eff'] and len(input_times) > 0:
        ax3.scatter(input_times, data['input_eff'][:, joint_idx], color='black', s=80, 
                    marker='s', zorder=5, label='Specified')
    ax3.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    ax3.set_ylabel('Effort [Nm]')
    ax3.set_xlabel('Time [s]')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    ax3.set_title('Effort', loc='left')
    
    # Waypoint time markers
    for t in input_times:
        for ax in axes:
            ax.axvline(x=t, color='gray', linestyle=':', alpha=0.4)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_error(log: TrajectoryLog, joint_idx: int = 0, save_path: Optional[str] = None):
    """
    Plot position error over time.
    """
    data = extract_data(log)
    if data is None:
        return None
    
    joint_name = log.joint_names[joint_idx] if joint_idx < len(log.joint_names) else f"Joint {joint_idx + 1}"
    
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle(f'{log.config_name} - Tracking Error\n{joint_name}', 
                 fontsize=12, fontweight='bold')
    
    times = data['times']
    input_times = data['input_times']
    
    # Position with feedback
    ax1 = axes[0]
    ax1.plot(times, data['pos'][: , joint_idx], '-', color='#1f77b4', linewidth=2, label='Reference')
    ax1.plot(times, data['fb_pos'][:, joint_idx], '--', color='#2ca02c', linewidth=1.5, label='Feedback')
    if len(input_times) > 0:
        ax1.scatter(input_times, data['input_pos'][:, joint_idx], color='black', s=80, 
                    marker='o', zorder=5, label='Waypoints')
    ax1.set_ylabel('Position [rad]')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Position:  Reference vs Feedback', loc='left')
    
    # Error
    ax2 = axes[1]
    ax2.plot(times, data['err_pos'][:, joint_idx], '-', color='#d62728', linewidth=2)
    ax2.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    ax2.fill_between(times, 0, data['err_pos'][:, joint_idx], alpha=0.3, color='#d62728')
    ax2.set_ylabel('Error [rad]')
    ax2.set_xlabel('Time [s]')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Position Error (Reference - Feedback)', loc='left')
    
    # Waypoint time markers
    for t in input_times:
        for ax in axes:
            ax.axvline(x=t, color='gray', linestyle=':', alpha=0.4)
    
    plt.tight_layout()
    
    if save_path: 
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved:  {save_path}")
    
    return fig


def main():
    parser = argparse.ArgumentParser(
        description='Visualize trajectory interpolation logs (documentation style with effort)')
    parser.add_argument('log_file', nargs='?', default=None,
                       help='Log file to parse.  If not provided, reads from stdin.')
    parser.add_argument('--save', '-s', type=str, default=None,
                       help='Save plots to files with this prefix')
    parser.add_argument('--no-show', action='store_true',
                       help='Do not display plots (only save)')
    parser.add_argument('--joint', '-j', type=int, default=0,
                       help='Joint index for single-joint plots (default: 0)')
    parser.add_argument('--list', '-l', action='store_true',
                       help='List found configurations without plotting')
    parser.add_argument('--style', type=str, default='doc',
                       choices=['doc', 'all', 'compare', 'phase', 'cmd', 'error'],
                       help='Plot style:  doc (single joint), all (all joints), compare, phase, cmd (command vs reference), error')
    args = parser.parse_args()
    
    # Read log data
    if args.log_file: 
        print(f"Reading from file: {args.log_file}")
        with open(args.log_file, 'r') as f:
            log_text = f.read()
    else:
        print("Reading from stdin (paste log and press Ctrl+D when done)...")
        log_text = sys.stdin.read()
    
    # Parse logs
    logs = parse_log(log_text)
    
    if not logs:
        print("No trajectory logs found!")
        print("\nExpected format with lines like:")
        print("  CONFIG:  TWO_POINT_SPLINE")
        print("  INTERPOLATION: splines")
        print("  --- INPUT_TRAJECTORY_START ---")
        print("  --- SAMPLED_TRAJECTORY_START ---")
        print("\nThe script handles CTest output format (lines starting with '3:  ' etc.)")
        return 1
    
    print(f"\nFound {len(logs)} trajectory log(s):")
    for i, log in enumerate(logs):
        has_vel = any(p.velocities for p in log.input_points)
        has_eff = any(p.efforts for p in log.input_points)
        spec_str = []
        if has_vel: 
            spec_str.append("vel")
        if has_eff:
            spec_str.append("eff")
        spec_info = f" [specified: {', '.join(spec_str)}]" if spec_str else ""
        print(f"  {i+1}. {log.config_name} ({log.interpolation_method}): "
              f"{len(log.input_points)} waypoints, {len(log.sampled_points)} samples{spec_info}")
    
    if args.list:
        return 0
    
    # Generate plots based on style
    if args.style == 'compare' and len(logs) >= 2:
        save_path = f"{args.save}_comparison. png" if args.save else None
        plot_comparison(logs, args.joint, save_path)
    else:
        for i, log in enumerate(logs):
            print(f"\nPlotting:  {log.config_name}")
            
            safe_name = re.sub(r'[^\w\-_]', '_', log.config_name)
            
            if args.style == 'doc':
                save_path = f"{args.save}_{safe_name}_joint{args.joint}.png" if args.save else None
                plot_documentation_style(log, args.joint, save_path)
            elif args.style == 'all': 
                save_path = f"{args.save}_{safe_name}_all.png" if args.save else None
                plot_all_joints(log, save_path)
            elif args. style == 'phase':
                save_path = f"{args.save}_{safe_name}_phase.png" if args.save else None
                plot_phase_portrait(log, save_path)
            elif args.style == 'cmd':
                save_path = f"{args.save}_{safe_name}_cmd. png" if args.save else None
                plot_command_vs_reference(log, args.joint, save_path)
            elif args.style == 'error': 
                save_path = f"{args.save}_{safe_name}_error.png" if args.save else None
                plot_error(log, args.joint, save_path)
    
    if not args.no_show:
        plt.show()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
