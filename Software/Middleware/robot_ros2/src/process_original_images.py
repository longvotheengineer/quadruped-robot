import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

def process_image(filename, title, ymin, ymax, legend_info):
    img_path = f"thesis/presentation/images/results/{filename}_backup.png"
    if not os.path.exists(img_path):
        print(f"File not found: {img_path}")
        return
        
    img = cv2.imread(img_path)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # Crop out the PlotJuggler axes and margins
    crop_x1 = 45
    crop_x2 = 1910
    crop_y1 = 20
    crop_y2 = 1140
    cropped = img_rgb[crop_y1:crop_y2, crop_x1:crop_x2]
    
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.imshow(cropped, extent=[0, 5, ymin, ymax], aspect='auto', interpolation='lanczos')
    
    ax.set_facecolor('#ffffff')
    fig.patch.set_facecolor('#ffffff')
    ax.tick_params(colors='black', labelsize=14)
    ax.set_title(title, color='black', fontsize=18, fontweight='bold', pad=15)
    ax.set_xlabel('Time (s)', color='black', fontsize=16)
    
    if "p_i_d" in filename or "sat_lpf" in filename:
        ax.set_ylabel('Correction Output', color='black', fontsize=16)
    else:
        ax.set_ylabel('Angle (rad)', color='black', fontsize=16)
    
    for label, color in legend_info:
        ax.plot([], [], color=color, lw=3, label=label)
        
    ax.legend(fontsize=14, loc='upper right', bbox_to_anchor=(1, 1), borderaxespad=0, facecolor='white', framealpha=0.9, edgecolor='black')
    
    fig.tight_layout()
    out_path = f"thesis/presentation/images/results/{filename}.png"
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"Saved {out_path}")

def main():
    # PITCH
    legend_pitch_meas_avg = [("Measurement", "#9467bd"), ("Moving Average", "#17becf")]
    process_image('trot_pid_pitch_controller_meas_avg_zoom', 'Pitch Controller: Measurement vs Moving Average', 
                  -0.15, 0.15, legend_pitch_meas_avg)
                  
    legend_pitch_pid = [("Proportional (P)", "#bcbd22"), ("Integral (I)", "#d62728"), ("Derivative (D)", "#1f77b4")]
    process_image('trot_pid_pitch_controller_p_i_d_zoom', 'Pitch Controller: P-I-D Components', 
                  -0.2, 0.2, legend_pitch_pid)
                  
    legend_pitch_sat = [("Saturated PID", "#ff7f0e"), ("After LPF", "#e377c2"), ("Feed-Forward", "#2ca02c")]
    process_image('trot_pid_pitch_controller_sat_lpf_ff_zoom', 'Pitch Controller: Saturation, LPF & FF', 
                  -0.35, 0.35, legend_pitch_sat)

    # ROLL
    legend_roll_meas_avg = [("Measurement", "#d62728"), ("Moving Average", "#2ca02c")]
    process_image('trot_pid_roll_controller_meas_avg_zoom', 'Roll Controller: Measurement vs Moving Average', 
                  -0.15, 0.15, legend_roll_meas_avg)

    legend_roll_pid = [("Proportional (P)", "#bcbd22"), ("Integral (I)", "#1f77b4"), ("Derivative (D)", "#17becf")]
    process_image('trot_pid_roll_controller_p_i_d_zoom', 'Roll Controller: P-I-D Components', 
                  -0.2, 0.2, legend_roll_pid)

    legend_roll_sat = [("Saturated PID", "#ff7f0e"), ("After LPF", "#e377c2"), ("Feed-Forward", "#9467bd")]
    process_image('trot_pid_roll_controller_sat_lpf_ff_zoom', 'Roll Controller: Saturation, LPF & FF', 
                  -0.35, 0.35, legend_roll_sat)

if __name__ == '__main__':
    main()
