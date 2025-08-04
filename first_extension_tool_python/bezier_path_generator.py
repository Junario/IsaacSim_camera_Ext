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

import math
import random
from pxr import Gf


class BezierPathGenerator:
    """부드러운 곡선 경로 생성기 - 체크포인트들을 자연스럽게 연결하는 연속적인 곡선"""
    
    def __init__(self, deviation_factor=0.05, speed_variation=0.05):
        """
        초기화
        
        Args:
            deviation_factor (float): 경로 편차 정도 (0.0 ~ 1.0) - 매우 작게 설정
            speed_variation (float): 속도 변동 정도 (0.0 ~ 1.0) - 매우 작게 설정
        """
        self.deviation_factor = deviation_factor
        self.speed_variation = speed_variation
        self.path_points = []  # 생성된 경로 포인트들
        self.current_path_index = 0
    
    def generate_bezier_path(self, checkpoints: list, points_per_segment=15):
        """
        체크포인트들을 부드러운 곡선으로 연결하는 경로 생성
        각 체크포인트를 자연스럽게 통과하는 연속적인 곡선
        
        Args:
            checkpoints (list): 체크포인트 위치 리스트
            points_per_segment (int): 세그먼트당 생성할 포인트 수
        
        Returns:
            list: 부드러운 곡선 경로 포인트들
        """
        if len(checkpoints) < 2:
            return checkpoints
        
        self.path_points = []
        self.checkpoints = checkpoints  # 체크포인트 정보 저장
        
        # 부드러운 곡선 경로 생성
        self.path_points = self._generate_extrema_based_path(checkpoints, points_per_segment)
        
        print(f"부드러운 곡선 경로 생성: {len(self.path_points)} 개의 포인트")
        if len(self.path_points) > 0:
            print(f"첫 번째 포인트: {self.path_points[0]}")
            print(f"마지막 포인트: {self.path_points[-1]}")
            
            # 체크포인트와 경로 포인트 비교
            print(f"체크포인트 수: {len(checkpoints)}")
            for i, cp in enumerate(checkpoints):
                print(f"CP {i}: {cp}")
            
            # 경로의 최소/최대 좌표 출력
            if len(self.path_points) > 0:
                x_coords = [p[0] for p in self.path_points]
                y_coords = [p[1] for p in self.path_points]
                z_coords = [p[2] for p in self.path_points]
                print(f"경로 X 범위: {min(x_coords):.1f} ~ {max(x_coords):.1f}")
                print(f"경로 Y 범위: {min(y_coords):.1f} ~ {max(y_coords):.1f}")
                print(f"경로 Z 범위: {min(z_coords):.1f} ~ {max(z_coords):.1f}")
        
        return self.path_points
    
    def _generate_extrema_based_path(self, checkpoints: list, points_per_segment: int):
        """
        부드러운 곡선 경로 생성
        각 체크포인트를 자연스럽게 연결하는 연속적인 곡선
        
        Args:
            checkpoints (list): 체크포인트 위치 리스트
            points_per_segment (int): 세그먼트당 생성할 포인트 수
        
        Returns:
            list: 부드러운 곡선 경로 포인트들
        """
        points = []
        
        # 전체 경로를 하나의 연속적인 곡선으로 생성
        # 체크포인트들을 제어점으로 사용하여 부드러운 곡선 생성
        
        # 시작점 추가 (드론 초기 위치)
        if checkpoints:
            points.append(checkpoints[0])
        
        # 체크포인트가 2개 이상인 경우 곡선 생성
        if len(checkpoints) >= 2:
            # 전체 곡선을 위한 포인트 수 계산
            total_points = points_per_segment * (len(checkpoints) - 1)
            
            for i in range(total_points):
                t = i / (total_points - 1)
                
                # 체크포인트 인덱스 계산
                checkpoint_index = t * (len(checkpoints) - 1)
                current_index = int(checkpoint_index)
                next_index = min(current_index + 1, len(checkpoints) - 1)
                
                # 보간 계수 계산
                local_t = checkpoint_index - current_index
                
                # 현재 세그먼트의 체크포인트들
                p0 = checkpoints[current_index]
                p1 = checkpoints[next_index]
                
                # 다음 세그먼트의 체크포인트 (곡선 방향 결정)
                if next_index + 1 < len(checkpoints):
                    p2 = checkpoints[next_index + 1]
                    # 3점 베지어 곡선으로 부드러운 전환
                    point = self._interpolate_smooth_bezier(p0, p1, p2, local_t)
                else:
                    # 마지막 세그먼트는 선형 보간
                    point = p0 * (1 - local_t) + p1 * local_t
                
                # 미세한 자연스러운 편차 추가
                point = self._add_natural_deviation(point, t)
                
                points.append(point)
        
        return points
    

    

    

    
    def _interpolate_smooth_bezier(self, p0: Gf.Vec3f, p1: Gf.Vec3f, p2: Gf.Vec3f, t: float):
        """
        부드러운 3점 베지어 곡선 보간
        
        Args:
            p0 (Gf.Vec3f): 시작점
            p1 (Gf.Vec3f): 중간점 (통과할 체크포인트)
            p2 (Gf.Vec3f): 끝점
            t (float): 보간 매개변수 (0.0 ~ 1.0)
        
        Returns:
            Gf.Vec3f: 보간된 점
        """
        # 중간점 계산
        mid_point = (p0 + p2) * 0.5
        
        # 체크포인트 방향으로의 제어점 생성
        direction_to_p1 = (p1 - mid_point).GetNormalized()
        
        # 곡선 강도 (더 부드럽게)
        curve_strength = 0.8
        control_offset = direction_to_p1 * (p1 - mid_point).GetLength() * curve_strength
        control_point = p1 + control_offset
        
        # 3점 베지어 곡선 공식: B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
        point = (1 - t) * (1 - t) * p0 + 2 * (1 - t) * t * control_point + t * t * p2
        
        return point
    
    def _add_natural_deviation(self, point: Gf.Vec3f, t: float):
        """
        자연스러운 미세 편차 추가 (숙달된 조종자의 미세한 조정)
        
        Args:
            point (Gf.Vec3f): 원래 점
            t (float): 세그먼트 내 위치 (0.0 ~ 1.0)
        
        Returns:
            Gf.Vec3f: 편차가 추가된 점
        """
        # 체크포인트 근처에서는 편차를 최소화
        deviation_factor = 0.03 * (1 - abs(2 * t - 1))  # 중간에서 최대, 끝에서 최소
        
        # 매우 작은 랜덤 편차
        deviation_x = random.uniform(-deviation_factor, deviation_factor)
        deviation_y = random.uniform(-deviation_factor, deviation_factor)
        deviation_z = random.uniform(-deviation_factor, deviation_factor)
        
        return Gf.Vec3f(
            point[0] + deviation_x,
            point[1] + deviation_y,
            point[2] + deviation_z
        )
    
    def get_speed_profile(self, base_speed: float):
        """
        일정 속도 프로파일 생성 (초기 설정된 속도로 계속 유지)
        
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
            # 미세한 속도 변동만 추가 (매우 작게)
            variation = random.uniform(1.0 - 0.02, 1.0 + 0.02)  # ±2% 변동
            speed = base_speed * variation
            speeds.append(speed)
        
        print(f"일정 속도 프로파일 생성: {len(speeds)} 개의 속도값")
        if len(speeds) > 0:
            print(f"최소 속도: {min(speeds):.2f}, 최대 속도: {max(speeds):.2f}, 평균 속도: {sum(speeds)/len(speeds):.2f}")
        
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