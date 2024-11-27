import matplotlib.pyplot as plt
import pandas as pd

# 파일 경로 설정
file1 = '/home/ircvlab/MOTOR/angles_data.txt'
file2 = '/home/ircvlab/VIO/odometry.txt'

# 데이터 로드 함수 정의
def load_data(file_path):
    try:
        df = pd.read_csv(file_path, sep=" ", header=None, usecols=[0, 1, 2], names=["timestamp", "x", "y"])
        return df
    except FileNotFoundError:
        print(f"The specified file {file_path} was not found.")
        exit()
    except ValueError:
        print(f"Error reading the file {file_path}. Please check the format.")
        exit()
def load_data2(file_path):
    try:
        df = pd.read_csv(file_path, sep=" ", header=None, usecols=[0, 1, 2], names=["timestamp", "x", "y"])
        df['x'] = -df['x']
        return df
    except FileNotFoundError:
        print(f"The specified file {file_path} was not found.")
        exit()
    except ValueError:
        print(f"Error reading the file {file_path}. Please check the format.")
        exit()
# 두 개의 파일에서 데이터 로드
df1 = load_data(file1)
df2 = load_data2(file2)

# 데이터 타입 변환 및 정렬
df1['timestamp'] = pd.to_datetime(df1['timestamp'], unit='s')
df2['timestamp'] = pd.to_datetime(df2['timestamp'], unit='s')

# 겹치는 타임스탬프 범위 찾기
start_time = max(df1['timestamp'].min(), df2['timestamp'].min())
end_time = min(df1['timestamp'].max(), df2['timestamp'].max())

# 겹치는 범위 필터링
filtered_df2 = df2[(df2['timestamp'] >= start_time) & (df2['timestamp'] <= end_time)]

# 가장 가까운 이전 값 찾기 및 오차 계산
errors_x = []
errors_y = []

for index, row in filtered_df2.iterrows():
    # 해당 타임스탬프보다 이전의 file1 값 찾기
    previous_value = df1[df1['timestamp'] <= row['timestamp']].iloc[-1]  # 가장 최근의 이전 값
    error_x = abs(previous_value['x'] - row['x'])  # file1의 x와 file2의 x 오차
    error_y = abs(previous_value['y'] - row['y'])  # file1의 y와 file2의 y 오차
    
    errors_x.append(error_x)
    errors_y.append(error_y)

average_error_x = pd.Series(errors_x).mean()
average_error_y = pd.Series(errors_y).mean()

print(f"Average Error Rate for X values: {average_error_x:.4f}")
print(f"Average Error Rate for Y values: {average_error_y:.4f}")

# 플롯 생성
plt.figure(figsize=(12, 12))

# X 값 플롯
plt.subplot(2, 1, 1)
plt.plot(df1["timestamp"], df1["y"], marker='o', color='b', label='GT')
plt.plot(df2["timestamp"], df2["x"], marker='x', color='r', label='Estimated Yaw Angle')
plt.title("X Values Over Time")
plt.ylabel("X Value")
plt.grid(True)
plt.legend()

# Y 값 플롯
plt.subplot(2, 1, 2)
plt.plot(df1["timestamp"], df1["x"], marker='o', color='g', label='Y Value File 1')
plt.plot(df2["timestamp"], df2["y"], marker='x', color='orange', label='Y Value File 2')
plt.xlabel("Timestamp (seconds)")
plt.ylabel("Y Value")
plt.grid(True)
plt.legend()

plt.tight_layout()

# 플롯 저장 및 표시
plt.savefig('xy_comparison_plot.png')
plt.show()
