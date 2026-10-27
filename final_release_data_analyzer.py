import os
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# 0. 환경 설정 및 상수 정의
# ==========================================
TARGET_FOLDER = r"" #CSV 파일들이 저장된 경로

# 평가 구간 (왕복 누적 거리)
EVAL_START_M = 0.015 

# 임무 식별 4자리 코드 매핑 테이블
LABEL_MAP = {
    # [제어 ON, 무게 X, 0.7m]
    '2326': ('Control ON', '0.7m', 'Weight_X'),
    '2328': ('Control ON', '0.7m', 'Weight_X'),
    '2330': ('Control ON', '0.7m', 'Weight_X'),
    
    # [제어 ON, 무게 X, 1.0m]
    '2332': ('Control ON', '1.0m', 'Weight_X'),
    '2335': ('Control ON', '1.0m', 'Weight_X'),
    '2341': ('Control ON', '1.0m', 'Weight_X'),
    
    # [제어 ON, 무게 O, 0.7m]
    '2353': ('Control ON', '0.7m', 'Weight_O'), 
    '2356': ('Control ON', '0.7m', 'Weight_O'),
    '0028': ('Control ON', '0.7m', 'Weight_O'),
    
    # [제어 ON, 무게 O, 1.0m]
    '0000': ('Control ON', '1.0m', 'Weight_O'),
    '0003': ('Control ON', '1.0m', 'Weight_O'),
    '0006': ('Control ON', '1.0m', 'Weight_O'),
    
    # [비제어 OFF, 무게 X, 0.7m]
    '2221': ('Control OFF', '0.7m', 'Weight_X'),
    '2223': ('Control OFF', '0.7m', 'Weight_X'),
    '2225': ('Control OFF', '0.7m', 'Weight_X'),
    
    # [비제어 OFF, 무게 X, 1.0m]
    '2226': ('Control OFF', '1.0m', 'Weight_X'),
    '2229': ('Control OFF', '1.0m', 'Weight_X'),
    '2232': ('Control OFF', '1.0m', 'Weight_X'),
    
    # [비제어 OFF, 무게 O, 0.7m]
    '0013': ('Control OFF', '0.7m', 'Weight_O'),
    '0015': ('Control OFF', '0.7m', 'Weight_O'),
    '0017': ('Control OFF', '0.7m', 'Weight_O'),
    
    # [비제어 OFF, 무게 O, 1.0m]
    '0019': ('Control OFF', '1.0m', 'Weight_O'),
    '0021': ('Control OFF', '1.0m', 'Weight_O'),
    '0024': ('Control OFF', '1.0m', 'Weight_O'),
}

# ==========================================
# 1. 단일/다중 임무 자동 분리 및 전처리
# ==========================================
def segment_missions(df_raw):
    length = df_raw['Length'].values
    n = len(length)
    
    in_mission = False
    segments = []
    start_idx = 0
    
    for i in range(n):
        if not in_mission and length[i] > 0.016: # 1.5cm 탈출 기준
            in_mission = True
            start_idx = max(0, i - 1)
        elif in_mission:
            if length[i] <= 0.0155 and np.max(length[start_idx:i+1]) > 0.05:
                segments.append(df_raw.iloc[start_idx:i+1].copy().reset_index(drop=True))
                in_mission = False
                
    if in_mission and np.max(length[start_idx:n]) > 0.05:
        segments.append(df_raw.iloc[start_idx:n].copy().reset_index(drop=True))
        
    if len(segments) == 0:
        segments = [df_raw.copy().reset_index(drop=True)]
        
    return segments

def load_and_preprocess_csv(fpath):
    df_raw = pd.read_csv(fpath, comment='#', header=None)
    
    if not str(df_raw.iloc[0, 0]).replace('.', '', 1).isdigit():
        df_raw = df_raw.iloc[1:].reset_index(drop=True)

    df = pd.DataFrame()
    df['Time']     = df_raw[0].astype(float)
    df['dt']       = df_raw[1].astype(float)
    df['ThetaMag'] = df_raw[10].astype(float)
    df['Length']   = df_raw[11].astype(float)
    df['L_dot']    = df_raw[12].astype(float)
    df['MotorCmd'] = df_raw[16].astype(float) # u_i (MotorCmd)
    df['SMCState'] = df_raw[19].astype(float)

    df['ThetaMag_deg'] = df['ThetaMag'] * (180.0 / np.pi)

    split_dfs = segment_missions(df)
    
    processed_segments = []
    for seg_df in split_dfs:
        length_diff = np.abs(np.diff(seg_df['Length'], prepend=seg_df['Length'].iloc[0]))
        seg_df['CumDistance'] = seg_df['Length'].iloc[0] + np.cumsum(length_diff)
        processed_segments.append(seg_df)
        
    return processed_segments

# ==========================================
# 2. 지표 연산 , 시각화 생성
# ==========================================
def compute_metrics_and_plot(df_seg, seg_name, length_cond, weight_cond, ctrl_mode, save_dir):
    target_l = 0.7 if length_cond == '0.7m' else 1.0
    eval_end_m = (2.0 * target_l) - 0.015

    mask = (df_seg['CumDistance'] >= EVAL_START_M) & (df_seg['CumDistance'] <= eval_end_m)
    df_eval = df_seg[mask].copy()
    
    if df_eval.empty:
        return None

    dt_sec = df_eval['dt'] / 1000.0
    th_deg = df_eval['ThetaMag_deg'].values
    motor_cmd = df_eval['MotorCmd'].values  # u_i (속도 명령값)
    
    total_task_time_sec = np.sum(dt_sec)
    rmse = np.sqrt(np.mean(th_deg ** 2))
    peak = np.max(np.abs(th_deg))
    sway_energy = np.sum((th_deg ** 2) * dt_sec)
    
    # 수식 (38): E_ctrl = sum(u_i^2 * dt_i)
    ctrl_effort = np.sum((motor_cmd ** 2) * dt_sec)
    
    # ------------------------------------
    # 그래프 시각화
    # ------------------------------------
    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    
    title_weight = "Weight_O" if weight_cond == "Weight_O" else "Weight_X"
    title_str = f"File: {seg_name} | {ctrl_mode} | Length {length_cond} | {title_weight}"
    fig.suptitle(title_str, fontsize=12, fontweight='bold')

    axs[0].plot(df_seg['CumDistance'], df_seg['ThetaMag_deg'], label='Theta Mag (deg)', color='red' if 'ON' in ctrl_mode else 'deepskyblue', linewidth=0.8)
    axs[0].axhline(y=4.0, color='gray', linestyle=':', label='Threshold (4°)')
    axs[0].set_ylabel('Theta Mag [deg]')
    axs[0].grid(True, linestyle='--', alpha=0.6)
    axs[0].legend(loc='upper right')

    smc_binary = (df_seg['SMCState'] == 1).astype(int)
    axs[1].step(df_seg['CumDistance'], smc_binary, where='post', label='SMC Active (1/0)', color='green', linewidth=0.8)
    axs[1].set_xlabel('Cumulative Distance [m]')
    axs[1].set_ylabel('SMC State')
    axs[1].set_ylim([-0.2, 1.2])
    axs[1].set_yticks([0, 1])
    axs[1].grid(True, linestyle='--', alpha=0.6)
    axs[1].legend(loc='upper right')

    axs[0].axvspan(EVAL_START_M, eval_end_m, color='cyan', alpha=0.2, label='Eval Region')
    axs[1].axvspan(EVAL_START_M, eval_end_m, color='cyan', alpha=0.2)

    plt.tight_layout()
    save_path = os.path.join(save_dir, f"{seg_name}_plot.png")
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close(fig)

    return {
        'RMSE_deg': rmse,
        'Peak_deg': peak,
        'Sway_Energy': sway_energy,
        'Ctrl_Effort': ctrl_effort,
        'Task_Time_sec': total_task_time_sec
    }

# ==========================================
# 3. 메인 분석 및 시리얼 출력
# ==========================================
def run_analysis():
    if not os.path.exists(TARGET_FOLDER):
        print(f" 폴더를 찾을 수 없습니다: {TARGET_FOLDER}")
        return

    plot_dir = os.path.join(TARGET_FOLDER, "Mission_Plots")
    os.makedirs(plot_dir, exist_ok=True)

    raw_results = []

    for file in os.listdir(TARGET_FOLDER):
        if not file.lower().endswith('.csv'):
            continue
            
        fpath = os.path.join(TARGET_FOLDER, file)
        
        match = re.search(r'(\d{4})\.csv$', file, re.IGNORECASE)
        if not match:
            continue
            
        code_4digit = match.group(1)
        if code_4digit not in LABEL_MAP:
            continue

        ctrl_mode, length_cond, weight_cond = LABEL_MAP[code_4digit]
        
        try:
            segments = load_and_preprocess_csv(fpath)
            
            for idx, df_seg in enumerate(segments):
                seg_name = f"{file.split('.csv')[0]}_M{idx+1}" if len(segments) > 1 else file.split('.csv')[0]
                
                metrics = compute_metrics_and_plot(df_seg, seg_name, length_cond, weight_cond, ctrl_mode, plot_dir)
                if metrics is None:
                    continue
                
                res_entry = {
                    'FileName': seg_name,
                    'Control_Mode': ctrl_mode,
                    'Length_Cond': length_cond,
                    'Weight_Cond': weight_cond,
                    **metrics
                }
                raw_results.append(res_entry)
                    
        except Exception as e:
            print(f" 파일 처리 실패 ({file}): {e}")

    df_res = pd.DataFrame(raw_results)
    if df_res.empty:
        print(" 처리할 수 있는 유효 데이터가 없습니다.")
        return

    # -------------------------------------------------------------
    # 1. 개별 파일 세그먼트 데이터 시리얼 출력 (MotorCmd 적분 기반)
    # -------------------------------------------------------------
    print("\n" + "="*115)
    print(" [개별 파일 측정 지표]")
    print("="*115)
    
    df_display = df_res.copy()
    df_display['RMSE (deg)'] = df_display['RMSE_deg'].map('{:.2f}'.format)
    df_display['Peak (deg)'] = df_display['Peak_deg'].map('{:.2f}'.format)
    df_display['Sway Energy'] = df_display['Sway_Energy'].map('{:.1f}'.format)
    df_display['Ctrl Effort (cmd²·s)'] = df_display['Ctrl_Effort'].map('{:.4f}'.format)
    df_display['Task Time (s)'] = df_display['Task_Time_sec'].map('{:.2f}'.format)

    print_cols = ['FileName', 'Control_Mode', 'Length_Cond', 'Weight_Cond', 'RMSE (deg)', 'Peak (deg)', 'Sway Energy', 'Ctrl Effort (cmd²·s)', 'Task Time (s)']
    print(df_display[print_cols].to_string(index=False))

    # -------------------------------------------------------------
    # 2. 4가지 시나리오별 평균 비교표 출력 (MotorCmd 적분 기반)
    # -------------------------------------------------------------
    grouped = df_res.groupby(['Length_Cond', 'Weight_Cond', 'Control_Mode'])[[
        'RMSE_deg', 'Peak_deg', 'Sway_Energy', 'Ctrl_Effort', 'Task_Time_sec'
    ]].mean().reset_index()

    print("\n\n" + "="*115)
    print(" [4가지 시나리오별 제어 vs 비제어 성능 비교표 (평균)]")
    print("="*115)

    scenarios = [
        ('0.7m', 'Weight_X'),
        ('0.7m', 'Weight_O'),
        ('1.0m', 'Weight_X'),
        ('1.0m', 'Weight_O'),
    ]

    compare_results = []
    
    for length, weight in scenarios:
        sub_df = grouped[(grouped['Length_Cond'] == length) & (grouped['Weight_Cond'] == weight)]
        
        if len(sub_df) < 2:
            continue
            
        on_row = sub_df[sub_df['Control_Mode'] == 'Control ON'].iloc[0]
        off_row = sub_df[sub_df['Control_Mode'] == 'Control OFF'].iloc[0]
        
        def calc_improvement(off_val, on_val):
            return ((off_val - on_val) / off_val) * 100.0

        scenario_name = f"L: {length}, W: {'O' if weight=='Weight_O' else 'X'}"
        
        compare_results.append({
            'Scenario': scenario_name,
            'Metric': 'RMSE (deg)',
            'Control OFF': f"{off_row['RMSE_deg']:.2f}",
            'Control ON': f"{on_row['RMSE_deg']:.2f}",
            'Improvement (%)': f"{calc_improvement(off_row['RMSE_deg'], on_row['RMSE_deg']):.1f}%"
        })
        compare_results.append({
            'Scenario': '',
            'Metric': 'Peak Angle (deg)',
            'Control OFF': f"{off_row['Peak_deg']:.2f}",
            'Control ON': f"{on_row['Peak_deg']:.2f}",
            'Improvement (%)': f"{calc_improvement(off_row['Peak_deg'], on_row['Peak_deg']):.1f}%"
        })
        compare_results.append({
            'Scenario': '',
            'Metric': 'Sway Energy',
            'Control OFF': f"{off_row['Sway_Energy']:.1f}",
            'Control ON': f"{on_row['Sway_Energy']:.1f}",
            'Improvement (%)': f"{calc_improvement(off_row['Sway_Energy'], on_row['Sway_Energy']):.1f}%"
        })
        # 수식 (38) 정의에 맞춘 MotorCmd 적분 기반 Ctrl Effort
        compare_results.append({
            'Scenario': '',
            'Metric': 'Ctrl Effort (cmd²·s)',
            'Control OFF': f"{off_row['Ctrl_Effort']:.4f}",
            'Control ON': f"{on_row['Ctrl_Effort']:.4f}",
            'Improvement (%)': f"{calc_improvement(off_row['Ctrl_Effort'], on_row['Ctrl_Effort']):.1f}%"
        })
        compare_results.append({
            'Scenario': '',
            'Metric': 'Task Time (s)',
            'Control OFF': f"{off_row['Task_Time_sec']:.2f}",
            'Control ON': f"{on_row['Task_Time_sec']:.2f}",
            'Improvement (%)': f"{calc_improvement(off_row['Task_Time_sec'], on_row['Task_Time_sec']):.1f}%"
        })
        compare_results.append({'Scenario': '-'*15, 'Metric': '-'*15, 'Control OFF': '-'*15, 'Control ON': '-'*15, 'Improvement (%)': '-'*15})

    compare_df = pd.DataFrame(compare_results)
    print(compare_df.to_string(index=False))

    print(f"\n 임무별 그래프 이미지 저장 완료: {plot_dir}")

if __name__ == "__main__":
    run_analysis()