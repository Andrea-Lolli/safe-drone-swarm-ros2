import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

def generate_global_heatmaps():
    print("Search all log files")
    
    results_dir = os.path.expanduser('~/safe_ws/mission_results')
    search_path = os.path.join(results_dir, "safe_mission_log_drone*.csv")
    
    all_files = glob.glob(search_path)
    if not all_files:
        print(f"No CSV file found{results_dir}.")
        return

    df_list = [pd.read_csv(f) for f in all_files]
    df_master = pd.concat(df_list, ignore_index=True)
    
    all_x = df_master['wp_x'].values
    all_y = df_master['wp_y'].values

    df_signals = df_master[df_master['device_id'] != 'NONE']
    dispositivi_trovati = df_signals['device_id'].unique()
    
    print(f"Found {len(dispositivi_trovati)} target devices. Heatmap generation started...")

    df_tutti_i_punti = df_master[['wp_x', 'wp_y']].drop_duplicates().copy()

    for dev_id in dispositivi_trovati:
        df_dev = df_signals[df_signals['device_id'] == dev_id][['wp_x', 'wp_y', 'avg_rssi']]
        df_dev = df_dev.groupby(['wp_x', 'wp_y']).mean().reset_index()
        df_merged = pd.merge(df_tutti_i_punti, df_dev, on=['wp_x', 'wp_y'], how='left')
        df_merged['avg_rssi'] = df_merged['avg_rssi'].fillna(-100.0)
        x = df_merged['wp_x'].values
        y = df_merged['wp_y'].values
        rssi = df_merged['avg_rssi'].values
        
        if len(x) < 4:
            continue

        grid_x, grid_y = np.mgrid[min(all_x)-2:max(all_x)+2:100j, min(all_y)-2:max(all_y)+2:100j]
        grid_rssi = griddata((x, y), rssi, (grid_x, grid_y), method='cubic')

        plt.figure(figsize=(9, 7))
        heatmap = plt.contourf(grid_x, grid_y, grid_rssi, levels=30, cmap='jet', alpha=0.8)
        cbar = plt.colorbar(heatmap)
        cbar.set_label('Potenza Segnale RSSI (dBm)')

        plt.scatter(x, y, c='black', s=15, alpha=0.4, label='Rotta Sciame (Tutti i WP)')

        max_idx = np.argmax(rssi)
        best_x, best_y, max_rssi = x[max_idx], y[max_idx], rssi[max_idx]
        plt.scatter(best_x, best_y, c='white', marker='*', s=250, edgecolors='black', 
                    label=f'Target: {dev_id}\n({max_rssi:.1f} dBm)')

        plt.title(f'Heatmap di Ricerca Multi-Drone - Target: {dev_id}', fontweight='bold')
        plt.xlabel('Asse X (metri)')
        plt.ylabel('Asse Y (metri)')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.3)
        plt.tight_layout()

    plt.show()

if __name__ == '__main__':
    generate_global_heatmaps()