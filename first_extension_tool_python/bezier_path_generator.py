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
    """베지어 곡선 기반 자연스러운 드론 경로 생성기"""
    
    def __init__(self, deviation_factor=0.25, speed_variation=0.2):
        """
        초기화
        
        Args:
            deviation_factor (float): 경로 편차 정도 (0.0 ~ 1.0)
            speed_variation (float): 속도 변동 정도 (0.0 ~ 1.0)
        """
        self.deviation_factor = deviation_factor
        self.speed_variation = speed_variation
        self.path_points = []  # 생성된 경로 포인트들
        self.current_path_index = 0
    
    def generate_bezier_path(self, checkpoints: list, points_per_segment=20):
        """
        체크포인트들을 기반으로 베지어 곡선 경로 생성
        각 체크포인트를 정확히 통과하는 자연스러운 곡선
        
        Args:
            checkpoints (list): 체크포인트 위치 리스트
            points_per_segment (int): 세그먼트당 생성할 포인트 수
        
        Returns:
            list: 베지어 곡선 경로 포인트들
        """
        if len(checkpoints) < 2:
            return checkpoints
        
        self.path_points = []
        self.checkpoints = checkpoints  # 체크포인트 정보 저장
        
        # 체크포인트가 2개인 경우: 직선 경로
        if len(checkpoints) == 2:
            segment_points = self._generate_2point_bezier(checkpoints[0], checkpoints[1], points_per_segment)
            self.path_points.extend(segment_points)
        
        # 체크포인트가 3개 이상인 경우: 세그먼트별 베지어 곡선
        else:
            # 각 세그먼트별로 베지어 곡선 생성
            self.path_points = self._generate_segmented_bezier(checkpoints, points_per_segment)
        
        print(f"베지어 곡선 경로 생성: {len(self.path_points)} 개의 포인트")
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
    
    def _generate_segmented_bezier(self, checkpoints: list, points_per_segment: int):
        """
        연속적인 베지어 곡선 생성
        체크포인트들을 자연스럽게 통과하는 부드러운 곡선
        
        Args:
            checkpoints (list): 체크포인트 위치 리스트
            points_per_segment (int): 세그먼트당 생성할 포인트 수
        
        Returns:
            list: 베지어 곡선 포인트들
        """
        points = []
        
        # 전체 경로를 하나의 연속적인 곡선으로 생성
        total_points = points_per_segment * len(checkpoints)
        
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
                point = self._interpolate_3point_bezier(p0, p1, p2, local_t)
            else:
                # 마지막 세그먼트는 선형 보간
                point = p0 * (1 - local_t) + p1 * local_t
            
            # 미세한 랜덤 오차 추가
            point = self._add_micro_deviation(point)
            
            points.append(point)
        
        return points
    
    def _interpolate_3point_bezier(self, p0: Gf.Vec3f, p1: Gf.Vec3f, p2: Gf.Vec3f, t: float):
        """
        3점 베지어 곡선 보간 (부드러운 전환)
        
        Args:
            p0 (Gf.Vec3f): 시작점
            p1 (Gf.Vec3f): 중간점 (통과할 체크포인트)
            p2 (Gf.Vec3f): 끝점
            t (float): 보간 매개변수 (0.0 ~ 1.0)
        
        Returns:
            Gf.Vec3f: 보간된 점
        """
        # 체크포인트 근처에서 더 정확하게 통과하도록 조정
        if t > 0.4 and t < 0.6:
            # 체크포인트 근처에서는 체크포인트에 더 가깝게 이동
            checkpoint_weight = 0.7
            point = p0 * (1 - t) * (1 - checkpoint_weight) + p1 * checkpoint_weight + p2 * t * (1 - checkpoint_weight)
        else:
            # 일반적인 3점 베지어 곡선
            mid_point = (p0 + p2) * 0.5
            direction_to_p1 = (p1 - mid_point).GetNormalized()
            
            # 제어점 생성 (체크포인트 방향으로)
            curve_strength = 0.6  # 곡선 강도 감소
            control_offset = direction_to_p1 * (p1 - mid_point).GetLength() * curve_strength
            control_point = p1 + control_offset
            
            # 3점 베지어 곡선 공식
            point = (1 - t) * (1 - t) * p0 + 2 * (1 - t) * t * control_point + t * t * p2
        
        return point
    
    def _bezier_curve_point(self, start_point: Gf.Vec3f, control_points: list, end_point: Gf.Vec3f, t: float):
        """
        베르슈타인 다항식을 사용한 베지어 곡선 점 계산
        
        Args:
            start_point (Gf.Vec3f): 시작점
            control_points (list): 제어점들
            end_point (Gf.Vec3f): 끝점
            t (float): 매개변수 (0.0 ~ 1.0)
        
        Returns:
            Gf.Vec3f: 베지어 곡선 상의 점
        """
        all_points = [start_point] + control_points + [end_point]
        n = len(all_points) - 1  # 곡선의 차수
        
        result = Gf.Vec3f(0.0, 0.0, 0.0)
        
        for i in range(n + 1):
            # 베르슈타인 다항식 계수
            coefficient = self._binomial_coefficient(n, i) * (t ** i) * ((1 - t) ** (n - i))
            # Vec3f 타입으로 명시적 변환
            point_vec3f = Gf.Vec3f(all_points[i][0], all_points[i][1], all_points[i][2])
            result += point_vec3f * coefficient
        
        return result
    
    def _binomial_coefficient(self, n: int, k: int):
        """이항 계수 계산"""
        if k > n:
            return 0
        if k == 0 or k == n:
            return 1
        
        result = 1
        for i in range(min(k, n - k)):
            result = result * (n - i) // (i + 1)
        
        return result
    
    def _generate_3point_bezier(self, p0: Gf.Vec3f, p1: Gf.Vec3f, p2: Gf.Vec3f, num_points: int):
        """
        3점 베지어 곡선 생성 (체크포인트를 정확히 통과)
        
        Args:
            p0 (Gf.Vec3f): 시작점
            p1 (Gf.Vec3f): 중간 체크포인트 (정확히 통과해야 함)
            p2 (Gf.Vec3f): 끝점
            num_points (int): 생성할 포인트 수
        
        Returns:
            list: 베지어 곡선 포인트들
        """
        points = []
        
        # 중간 체크포인트를 정확히 통과하도록 제어점 조정
        # 시작점과 끝점을 잇는 직선에서 중간 체크포인트 방향으로 제어점 생성
        mid_point = (p0 + p2) * 0.5
        direction_to_p1 = (p1 - mid_point).GetNormalized()
        
        # 중간 체크포인트를 향하는 제어점 생성
        curve_strength = 1.2  # 곡선 강도 증가
        control_offset = direction_to_p1 * (p1 - mid_point).GetLength() * curve_strength
        control_point = p1 + control_offset
        
        # 추가 편차로 더 자연스러운 경로
        control_point = self._add_deviation(control_point, p0, p2)
        
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # 3점 베지어 곡선 공식: B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
            point = (1 - t) * (1 - t) * p0 + 2 * (1 - t) * t * control_point + t * t * p2
            
            # 미세한 랜덤 오차 추가 (숙달된 조종자의 미세한 조정)
            point = self._add_micro_deviation(point)
            
            points.append(point)
        
        # 중간 체크포인트를 정확히 포함하도록 보장
        if num_points > 0:
            mid_index = num_points // 2
            points[mid_index] = p1  # 중간 지점을 정확한 체크포인트로 설정
        
        return points
    
    def _generate_2point_bezier(self, p0: Gf.Vec3f, p1: Gf.Vec3f, num_points: int):
        """
        2점 베지어 곡선 생성 (직선에 약간의 곡선 추가)
        
        Args:
            p0 (Gf.Vec3f): 시작점
            p1 (Gf.Vec3f): 끝점
            num_points (int): 생성할 포인트 수
        
        Returns:
            list: 베지어 곡선 포인트들
        """
        points = []
        
        # 중간 제어점 생성 (직선에서 약간 벗어난 점)
        mid_point = (p0 + p1) * 0.5
        control_point = self._add_deviation(mid_point, p0, p1)
        
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # 2점 베지어 곡선 공식: B(t) = (1-t)P₀ + tP₁ (제어점 추가)
            point = (1 - t) * p0 + t * p1
            
            # 제어점의 영향 추가
            control_influence = 0.3 * (1 - t) * t * (control_point - mid_point)
            point += control_influence
            
            # 미세한 랜덤 오차 추가
            point = self._add_micro_deviation(point)
            
            points.append(point)
        
        return points
    
    def _add_deviation(self, point: Gf.Vec3f, prev_point: Gf.Vec3f, next_point: Gf.Vec3f):
        """
        경로 편차 추가
        
        Args:
            point (Gf.Vec3f): 원래 점
            prev_point (Gf.Vec3f): 이전 점
            next_point (Gf.Vec3f): 다음 점
        
        Returns:
            Gf.Vec3f: 편차가 추가된 점
        """
        # 이전 점과 다음 점을 잇는 방향 벡터
        direction = (next_point - prev_point).GetNormalized()
        
        # 수직 방향 벡터 생성 (편차 방향)
        perpendicular = Gf.Vec3f(-direction[1], direction[0], direction[2])
        
        # 랜덤한 편차 크기
        deviation_magnitude = random.uniform(-self.deviation_factor, self.deviation_factor)
        
        # 편차 추가 (Vec3f 타입으로 명시적 변환)
        deviated_point = Gf.Vec3f(
            point[0] + perpendicular[0] * deviation_magnitude,
            point[1] + perpendicular[1] * deviation_magnitude,
            point[2] + perpendicular[2] * deviation_magnitude
        )
        
        return deviated_point
    
    def _add_micro_deviation(self, point: Gf.Vec3f):
        """
        미세한 랜덤 오차 추가 (숙달된 조종자의 미세한 조정)
        
        Args:
            point (Gf.Vec3f): 원래 점
        
        Returns:
            Gf.Vec3f: 미세한 오차가 추가된 점
        """
        # 매우 작은 랜덤 오차
        micro_deviation_x = random.uniform(-0.05, 0.05)
        micro_deviation_y = random.uniform(-0.05, 0.05)
        micro_deviation_z = random.uniform(-0.05, 0.05)
        
        # Vec3f 타입으로 명시적 변환
        return Gf.Vec3f(
            point[0] + micro_deviation_x,
            point[1] + micro_deviation_y,
            point[2] + micro_deviation_z
        )
    
    def get_speed_profile(self, base_speed: float):
        """
        체크포인트별 속도 프로파일 생성 (감속 -> 가속 패턴)
        
        Args:
            base_speed (float): 기본 속도
        
        Returns:
            list: 각 경로 포인트에 대한 속도
        """
        speeds = []
        num_points = len(self.path_points)
        
        if num_points == 0:
            return [base_speed]
        
        # 체크포인트별 세그먼트 정보 계산
        segment_info = self._calculate_segment_info()
        
        for i in range(num_points):
            # 현재 포인트가 어느 세그먼트에 속하는지 확인
            segment_index, local_t = self._get_segment_info(i, segment_info)
            
            # 체크포인트별 속도 프로파일 생성
            speed_factor = self._get_checkpoint_based_speed_factor(segment_index, local_t, len(segment_info))
            
            # 미세한 속도 변동 추가
            variation = random.uniform(1.0 - self.speed_variation, 1.0 + self.speed_variation)
            
            speed = base_speed * speed_factor * variation
            
            # 최소 속도 보장
            speed = max(speed, base_speed * 0.1)
            
            speeds.append(speed)
        
        print(f"체크포인트별 속도 프로파일 생성: {len(speeds)} 개의 속도값")
        if len(speeds) > 0:
            print(f"최소 속도: {min(speeds):.2f}, 최대 속도: {max(speeds):.2f}, 평균 속도: {sum(speeds)/len(speeds):.2f}")
        
        return speeds
    
    def _calculate_segment_info(self):
        """세그먼트 정보 계산 (각 체크포인트 구간별 정보)"""
        segment_info = []
        total_points = len(self.path_points)
        
        if total_points == 0:
            return segment_info
        
        # 각 세그먼트의 시작과 끝 인덱스 계산
        points_per_segment = total_points // max(1, len(self.checkpoints) - 1)
        
        for i in range(len(self.checkpoints) - 1):
            start_idx = i * points_per_segment
            end_idx = (i + 1) * points_per_segment if i < len(self.checkpoints) - 2 else total_points
            
            segment_info.append({
                'start_idx': start_idx,
                'end_idx': end_idx,
                'length': end_idx - start_idx
            })
        
        return segment_info
    
    def _get_segment_info(self, point_index: int, segment_info: list):
        """포인트 인덱스에 해당하는 세그먼트 정보 반환"""
        for i, segment in enumerate(segment_info):
            if segment['start_idx'] <= point_index < segment['end_idx']:
                local_t = (point_index - segment['start_idx']) / max(1, segment['length'] - 1)
                return i, local_t
        
        # 마지막 세그먼트에 속하는 경우
        if segment_info:
            last_segment = segment_info[-1]
            local_t = (point_index - last_segment['start_idx']) / max(1, last_segment['length'] - 1)
            return len(segment_info) - 1, local_t
        
        return 0, 0.0
    
    def _get_checkpoint_based_speed_factor(self, segment_index: int, local_t: float, total_segments: int):
        """
        체크포인트별 속도 팩터 계산 (감속 -> 가속 패턴)
        
        Args:
            segment_index (int): 현재 세그먼트 인덱스
            local_t (float): 세그먼트 내 위치 (0.0 ~ 1.0)
            total_segments (int): 총 세그먼트 수
        
        Returns:
            float: 속도 팩터 (0.0 ~ 1.0)
        """
        # 첫 번째 세그먼트: 시작 가속
        if segment_index == 0:
            if local_t < 0.3:  # 시작 시 가속
                return 0.3 + 0.7 * (local_t / 0.3)
            else:  # 일정 속도
                return 1.0
        
        # 마지막 세그먼트: 마지막 체크포인트에서 감속
        elif segment_index == total_segments - 1:
            if local_t > 0.7:  # 마지막 체크포인트 근처에서 감속
                return 1.0 - 0.7 * ((local_t - 0.7) / 0.3)
            else:  # 일정 속도
                return 1.0
        
        # 중간 세그먼트들: 감속 -> 가속 패턴
        else:
            if local_t < 0.4:  # 체크포인트 도달 전 감속
                return 1.0 - 0.5 * (local_t / 0.4)
            elif local_t < 0.6:  # 체크포인트 근처 최저 속도
                return 0.5
            else:  # 체크포인트 지나면서 가속
                return 0.5 + 0.5 * ((local_t - 0.6) / 0.4)
    
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