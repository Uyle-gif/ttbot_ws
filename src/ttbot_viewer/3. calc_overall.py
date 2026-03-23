import pandas as pd
import os

file_path = os.path.expanduser('~/ttbot_ws/src/FAST-LIVO2/Log/odom_time.csv')
df = pd.read_csv(file_path)
df['Latency_ms'] = pd.to_numeric(df['Latency_ms'], errors='coerce').dropna()

print(f"1. Overall Mean  : {df['Latency_ms'].mean():.2f} ms")
print(f"2. Worst-case Max: {df['Latency_ms'].max():.2f} ms")
print(f"3. Jitter (Std)  : {df['Latency_ms'].std():.2f} ms")