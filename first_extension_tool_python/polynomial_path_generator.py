# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import math
from pxr import Gf
from scipy.interpolate import CubicSpline, interp1d


class PolynomialPathGenerator:
    """체크포인트들을 극점으로 하는 매끄러운 n차 다항식 곡선 생성기"""
    
    def __init__(self, points_per_segment=20):
        """
        초기화
        
        Args:
            points_per_segment (int): 세그먼트당 생성할 포인트 수
        """
        self.points_per_segment = points_per_segment
        self.path_points = []
        self.checkpoints = []
    
    def generate_polynomial_path(self, checkpoints: list):
        """
        체크포인트들을 극점으로 하는 매끄러운 다항식 곡선 생성
        
        Args:
            checkpoints (list): 체크포인트 위치 리스트 (극점들)
        
        Returns:
            list: 매끄러운 다항식 곡선 경로 포인트들
        """
        if len(checkpoints) < 2:
            return checkpoints
        
        self.checkpoints = checkpoints
        print(f"=== 다항식 곡선 경로 생성 시작 ===")
        print(f"체크포인트 수: {len(checkpoints)}")
        for i, cp in enumerate(checkpoints):
            print(f"CP {i}: {cp}")
        
        # 매개변수화된 곡선 생성
        self.path_points = self._generate_parametric_curve(checkpoints)
        
        print(f"다항식 곡선 경로 생성 완료: {len(self.path_points)} 개의 포인트")
        if len(self.path_points) > 0:
            print(f"첫 번째 포인트: {self.path_points[0]}")
            print(f"마지막 포인트: {self.path_points[-1]}")
            
            # 경로의 최소/최대 좌표 출력
            x_coords = [p[0] for p in self.path_points]
            y_coords = [p[1] for p in self.path_points]
            z_coords = [p[2] for p in self.path_points]
            print(f"경로 X 범위: {min(x_coords):.1f} ~ {max(x_coords):.1f}")
            print(f"경로 Y 범위: {min(y_coords):.1f} ~ {max(y_coords):.1f}")
            print(f"경로 Z 범위: {min(z_coords):.1f} ~ {max(z_coords):.1f}")
        
        return self.path_points
    
    def _generate_parametric_curve(self, checkpoints: list):
        """
        매개변수화된 곡선 생성 (체크포인트들을 극점으로 하는 연속적인 곡선)
        
        Args:
            checkpoints (list): 체크포인트 위치 리스트
        
        Returns:
            list: 매끄러운 곡선 경로 포인트들
        """
        if len(checkpoints) < 2:
            return checkpoints
        
        # 체크포인트를 numpy 배열로 변환
        points = np.array([[cp[0], cp[1], cp[2]] for cp in checkpoints])
        
        # 매개변수 t 생성 (0부터 1까지 균등 분포)
        t = np.linspace(0, 1, len(checkpoints))
        
        # 각 좌표축에 대해 스플라인 보간 수행
        try:
            # X 좌표 스플라인
            spline_x = CubicSpline(t, points[:, 0], bc_type='periodic')
            # Y 좌표 스플라인
            spline_y = CubicSpline(t, points[:, 1], bc_type='periodic')
            # Z 좌표 스플라인
            spline_z = CubicSpline(t, points[:, 2], bc_type='periodic')
            
            print("주기적 스플라인 사용 (원형 경로)")
        except:
            # 주기적 조건이 실패하면 자연 경계 조건 사용
            try:
                spline_x = CubicSpline(t, points[:, 0], bc_type='natural')
                spline_y = CubicSpline(t, points[:, 1], bc_type='natural')
                spline_z = CubicSpline(t, points[:, 2], bc_type='natural')
                print("자연 경계 스플라인 사용")
            except:
                # 스플라인이 실패하면 선형 보간 사용
                spline_x = interp1d(t, points[:, 0], kind='linear')
                spline_y = interp1d(t, points[:, 1], kind='linear')
                spline_z = interp1d(t, points[:, 2], kind='linear')
                print("선형 보간 사용")
        
        # 더 조밀한 매개변수 생성 (더 부드러운 곡선)
        num_points = len(checkpoints) * self.points_per_segment
        t_dense = np.linspace(0, 1, num_points)
        
        # 곡선 포인트 생성
        curve_points = []
        for t_val in t_dense:
            x = float(spline_x(t_val))
            y = float(spline_y(t_val))
            z = float(spline_z(t_val))
            curve_points.append(Gf.Vec3f(x, y, z))
        
        # 시작점과 끝점이 체크포인트와 정확히 일치하도록 조정
        if curve_points:
            curve_points[0] = checkpoints[0]
            curve_points[-1] = checkpoints[-1]
        
        return curve_points
    
    def get_speed_profile(self, base_speed: float):
        """
        일정 속도 프로파일 생성
        
        Args:
            base_speed (float): 기본 속도
        
        Returns:
            list: 각 경로 포인트에 대한 속도
        """
        speeds = []
        num_points = len(self.path_points)
        
        if num_points == 0:
            return [base_speed]
        
        # 모든 경로 포인트에 대해 일정한 속도 적용
        for i in range(num_points):
            speeds.append(base_speed)
        
        print(f"일정 속도 프로파일 생성: {len(speeds)} 개의 속도값")
        print(f"속도: {base_speed:.2f} m/s")
        
        return speeds
    
    def get_next_target(self, current_position: Gf.Vec3f, look_ahead_distance: float = 1.0):
        """
        현재 위치에서 다음 목표 지점 찾기
        
        Args:
            current_position (Gf.Vec3f): 현재 위치
            look_ahead_distance (float): 앞을 보는 거리
        
        Returns:
            tuple: (다음 목표 지점, 도달 여부)
        """
        if not hasattr(self, 'current_path_index'):
            self.current_path_index = 0
        
        if self.current_path_index >= len(self.path_points):
            return None, True
        
        # 현재 경로 포인트에서 look_ahead_distance만큼 앞의 지점 찾기
        target_index = min(self.current_path_index + int(look_ahead_distance * 10), len(self.path_points) - 1)
        target_point = self.path_points[target_index]
        
        # 현재 위치에서 목표 지점까지의 거리
        distance = (target_point - current_position).GetLength()
        
        # 목표 지점에 충분히 가까우면 다음 인덱스로 이동
        if distance < 0.3:
            self.current_path_index = min(self.current_path_index + 1, len(self.path_points) - 1)
        
        # 경로의 끝에 도달했는지 확인
        reached_end = self.current_path_index >= len(self.path_points) - 1
        
        return target_point, reached_end
    
    def reset_path_index(self):
        """경로 인덱스 초기화"""
        self.current_path_index = 0 