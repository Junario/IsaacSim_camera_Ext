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
    """극값(체크포인트)에서 기울기가 0이 되었다가 방향이 바뀌는 곡선 경로 생성기"""
    
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
        체크포인트들을 극값처럼 처리하는 곡선 경로 생성
        각 체크포인트에서 기울기가 0이 되었다가 방향이 바뀌는 곡선
        
        Args:
            checkpoints (list): 체크포인트 위치 리스트
            points_per_segment (int): 세그먼트당 생성할 포인트 수
        
        Returns:
            list: 곡선 경로 포인트들
        """
        if len(checkpoints) < 2:
            return checkpoints
        
        self.path_points = []
        self.checkpoints = checkpoints  # 체크포인트 정보 저장
        
        # 극값 기반 곡선 경로 생성
        self.path_points = self._generate_extrema_based_path(checkpoints, points_per_segment)
        
        print(f"극값 기반 곡선 경로 생성: {len(self.path_points)} 개의 포인트")
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
        극값 기반 곡선 경로 생성
        각 체크포인트에서 기울기가 0이 되었다가 방향이 바뀌는 곡선
        
        Args:
            checkpoints (list): 체크포인트 위치 리스트
            points_per_segment (int): 세그먼트당 생성할 포인트 수
        
        Returns:
            list: 곡선 경로 포인트들
        """
        points = []
        
        # 각 체크포인트 쌍 사이에 극값 기반 곡선 생성
        for i in range(len(checkpoints) - 1):
            start_cp = checkpoints[i]
            end_cp = checkpoints[i + 1]
            
            # 현재 세그먼트의 극값 기반 곡선 생성
            segment_points = self._generate_extrema_segment(start_cp, end_cp, points_per_segment)
            
            # 첫 번째 세그먼트가 아닌 경우 시작점 제거 (중복 방지)
            if i > 0:
                segment_points = segment_points[1:]
            
            points.extend(segment_points)
        
        # 마지막 체크포인트도 포함
        if checkpoints:
            points.append(checkpoints[-1])
        
        return points
    
    def _generate_extrema_segment(self, start_point: Gf.Vec3f, end_point: Gf.Vec3f, num_points: int):
        """
        두 체크포인트 사이의 극값 기반 곡선 세그먼트 생성
        시작점에서 감속 -> 체크포인트에서 정지 -> 끝점으로 가속
        
        Args:
            start_point (Gf.Vec3f): 시작 체크포인트
            end_point (Gf.Vec3f): 끝 체크포인트
            num_points (int): 생성할 포인트 수
        
        Returns:
            list: 극값 기반 곡선 세그먼트 포인트들
        """
        points = []
        
        # 방향 벡터와 거리 계산
        direction = (end_point - start_point).GetNormalized()
        distance = (end_point - start_point).GetLength()
        
        # 중간 제어점 생성 (곡선의 방향 결정)
        mid_point = (start_point + end_point) * 0.5
        
        # 극값 기반 곡선을 위한 추가 제어점들
        control_points = self._generate_extrema_control_points(start_point, end_point, direction, distance)
        
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # 극값 기반 곡선 보간 (3차 다항식 스타일)
            point = self._interpolate_extrema_curve(start_point, end_point, control_points, t)
            
            # 미세한 자연스러운 편차 추가
            point = self._add_natural_deviation(point, t)
            
            points.append(point)
        
        # 시작점과 끝점을 정확히 포함
        if points:
            points[0] = start_point
            points[-1] = end_point
        
        return points
    
    def _generate_extrema_control_points(self, start_point: Gf.Vec3f, end_point: Gf.Vec3f, direction: Gf.Vec3f, distance: float):
        """
        극값 기반 곡선을 위한 제어점들 생성
        
        Args:
            start_point (Gf.Vec3f): 시작점
            end_point (Gf.Vec3f): 끝점
            direction (Gf.Vec3f): 방향 벡터
            distance (float): 거리
        
        Returns:
            list: 제어점들
        """
        control_points = []
        
        # 수직 방향 벡터 생성 (곡선 방향)
        perpendicular = Gf.Vec3f(-direction[1], direction[0], direction[2])
        
        # 중간점
        mid_point = (start_point + end_point) * 0.5
        
        # 극값 효과를 위한 제어점들
        # 1. 시작점 근처의 제어점 (감속 구간)
        decel_control = start_point + direction * (distance * 0.3) + perpendicular * (distance * 0.2)
        
        # 2. 중간 극값 제어점 (정지 구간)
        extrema_control = mid_point + perpendicular * (distance * 0.3)
        
        # 3. 끝점 근처의 제어점 (가속 구간)
        accel_control = end_point - direction * (distance * 0.3) + perpendicular * (distance * 0.2)
        
        control_points = [decel_control, extrema_control, accel_control]
        
        return control_points
    
    def _interpolate_extrema_curve(self, start_point: Gf.Vec3f, end_point: Gf.Vec3f, control_points: list, t: float):
        """
        극값 기반 곡선 보간 (3차 다항식 스타일)
        
        Args:
            start_point (Gf.Vec3f): 시작점
            end_point (Gf.Vec3f): 끝점
            control_points (list): 제어점들
            t (float): 보간 매개변수 (0.0 ~ 1.0)
        
        Returns:
            Gf.Vec3f: 보간된 점
        """
        # 극값 효과를 위한 수정된 베지어 곡선
        # 시작점에서 감속 -> 중간에서 정지 -> 끝점으로 가속
        
        if len(control_points) >= 3:
            cp1, cp2, cp3 = control_points[0], control_points[1], control_points[2]
            
            # 중간점 계산
            mid_point = (start_point + end_point) * 0.5
            
            # 극값 효과를 위한 수정된 4점 베지어 곡선
            # B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
            
            # 극값 효과를 위한 수정된 가중치
            if t < 0.5:
                # 전반부: 감속 구간
                weight = 2 * t  # 0 -> 1
                point = (1 - weight) * start_point + weight * cp1
            else:
                # 후반부: 가속 구간
                weight = 2 * (t - 0.5)  # 0 -> 1
                point = (1 - weight) * cp3 + weight * end_point
            
            # 중간 극값 제어점의 영향 추가
            extrema_influence = 0.3 * (1 - abs(2 * t - 1)) * (cp2 - mid_point)
            point += extrema_influence
            
        else:
            # 기본 선형 보간
            point = start_point * (1 - t) + end_point * t
        
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