#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# 그래프 크기 설정
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')  # 3D 그래프 설정

# x, y 데이터 생성
x = np.linspace(-10, 10, 100)  # -10 ~ 10 범위의 100개 점
y = np.linspace(-10, 10, 100)
z = np.sin(np.sqrt(x**2 + y**2))  # 예제용 Z 값

# 3D 그래프 그리기 (선 그래프)
ax.plot(x, y, z, label="3D Line")

# 화살표를 그리는 함수 (X/Y/Z축)
ax.quiver(-10, 0, 0, 20, 0, 0, color='red', linewidth=3, arrow_length_ratio=0.1)  # X축 (빨간색)
ax.quiver(0, 0, 0, 0, 20, 0, color='blue', linewidth=3, arrow_length_ratio=0.1)  # Y축 (노란색) -> Z축
ax.quiver(0, 0, -10, 0, 0, 20, color='green', linewidth=3, arrow_length_ratio=0.1)  # Z축 (파란색) -> Y축

# x, y, z 축 범위 설정
ax.set_xlim([-10, 10])
ax.set_ylim([0, 20])
ax.set_zlim([10, -10])

# 축 제목 추가
ax.set_xlabel("X Axis")
ax.set_ylabel("Z Axis")
ax.set_zlabel("Y Axis")

# 제목 추가
ax.set_title("3D Graph Test")

# 범례 추가
ax.legend()

#plt.savefig("graph.png", dpi=300, bbox_inches='tight')
plt.show()
