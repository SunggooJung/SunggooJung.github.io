---
title: "LOAM paper and Source code Interpretation"
date: 2020-05-18 11:35:28 +0900
categories: jekyll update

---
## Lidar Odometry And Mapping (LOAM)$^{[*]}$
\*Zhang, J., & Singh, S. (2014, July). LOAM: Lidar Odometry and Mapping in Real-time. In _Robotics: Science and Systems_ (Vol. 2, No. 9).
### Spinning Hokuyo를 이용한 SLAM을 개발하기 위한 Study

#### 1. Notation

-  {$L$}: LiDAR 좌표계 (Body-Frame)

-  {$W$}: World 좌표계 ({$L$}의 Initial Position, Map-Frame)
    
-  $P_{k}$: k번째 sweep을 통해 받은 Point Cloud 집합    

-  $X^L_{(k,i)}$: LiDAR 좌표계에서 $P_{k}$안의 특정 포인트 i의 좌표, $i \in P_{k}$

-  $X^W_{(k,i)}$: World 좌표계에서 $P_{k}$안의 특정 포인트 i의 좌표, $i \in P_{k}$

- $S$ : 동일 scan내에서 얻어지는 특정 point *i*와 그 주변 point의 집합 (논문에서는 *i*의 앞뒤로 5개씩의 point를 선정, 한 scan은 coplanar로 가정)
    

#### 2. Feature Point Extraction
Laser Odometry 측정 방식은 각 frame의 pointcloud를 이전 frame의 pointcloud와 일치시킴으로써 두 frame 사이의 로봇의 상대 변위를 얻는다. 일반적인 방식은 **ICP (Iterative Closest Point)** 와 같이 Original pointcloud를 사용하여 frame matching을 수행하지만, LOAM은 특징점을 먼저 추출한 후 이 특징점만을 가지고 **feature point matching**을 수행한다.

Feature point란 특정 특성(certain characteristic)이 있는 점인데 LOAM에서는 아래 수식에 정의된 c값 (curvature, 곡률)에 따라 2가지의 Feature를 정의하고 있다. 

- Edge Point (c값이 큰점),  Planar Point (c값이 작은점)

추출된 특징점이 Edge인지, Planar인지 구분하는 방식은 다음의 $c$(curvature)를 계산함으로서 구할 수 있다.

$\Large c = \frac{1}{|S|\cdot\Vert X^L_{(k,i)}\Vert}\Vert\sum_{j \in S, i \neq j}(X^L_{(k,i)}-X^L_{(k,j)})\Vert$ 

즉 검사하고자하는 point *i*의 주변 point와의 차이를 합하고 이를  평균내어 *c*를 계산한다. 이때, $X^L_{(k,i)}$로  c를 나누는 것을 통해 현재위치로 부터 멀리있는 point일 수록 *c*의 값을 작게 하는 효과를 준다. 이로부터 *c*값이 크면 Edge, 작으면 Planar로 정의한다.

이를 구현한 LOAM 저자의 code는 다음과 같다. 
```c++
//curvature 계산 공식 중에서, 
//scan은 CW또는 CCW로 순차적으로 진행되기 때문에
//point i의 좌우로 각각5개씩 총 10개의 point와의 차이를 계산하고
//그 크기(curvature)를 pointcloud.s 에 저장
//직선일 경우 0, 곡률이 클 수록 c (diff)가 커짐
//i가 5부터 시작하는 이유는 scan의 가장자리 5개 point를 제외하고 특징점을 추출하기 때문
for (int i=5; i< cloudSize - 5; i++){

	float diffX = laserCloud->points[i-5].x + laserCloud->points[i-4].x +
		laserCloud->points[i-3].x + laserCloud->points[i-2].x +	
		laserCloud->points[i-1].x - 10* laserCloud->points[i].x +
		laserCloud->points[i+1].x + laserCloud->points[i+2].x +	
		laserCloud->points[i+3].x + laserCloud->points[i+4].x +	
		laserCloud->points[i+5].x;
		//=(laserCloud->points[i-5].x - laserCloud-points[i].x) + 
		// (laserCloud->points[i-4].x - laserCloud-points[i].x) + 
		// (laserCloud->points[i-3].x - laserCloud-points[i].x) + 
		// (laserCloud->points[i-2].x - laserCloud-points[i].x) + 
		// (laserCloud->points[i-1].x - laserCloud-points[i].x) + 
		// (laserCloud->points[i+1].x - laserCloud-points[i].x) +
		// (laserCloud->points[i+2].x - laserCloud-points[i].x) +
		// (laserCloud->points[i+3].x - laserCloud-points[i].x) +
		// (laserCloud->points[i+4].x - laserCloud-points[i].x) +
		// (laserCloud->points[i+5].x - laserCloud-points[i].x)
	
	float diffY = laserCloud->points[i-5].y + laserCloud->points[i-4].y +
		laserCloud->points[i-3].y + laserCloud->points[i-2].y +	
		laserCloud->points[i-1].y - 10* laserCloud->points[i].y +
		laserCloud->points[i+1].y + laserCloud->points[i+2].y +	
		laserCloud->points[i+3].y + laserCloud->points[i+4].y +	
		laserCloud->points[i+5].y;
	
	float diffZ = laserCloud->points[i-5].z + laserCloud->points[i-4].z +
		laserCloud->points[i-3].z + laserCloud->points[i-2].z +	
		laserCloud->points[i-1].z - 10* laserCloud->points[i].z +
		laserCloud->points[i+1].z + laserCloud->points[i+2].z +	
		laserCloud->points[i+3].z + laserCloud->points[i+4].z +	
		laserCloud->points[i+5].z;
	
	laserCloud->points[i].s = diffX*diffX + diffY*diffY + diffZ*diffZ;
}
```
이렇게 curvature를 계산하고 난 후에는 이 feature point를 사용하여 두 frame사이의 상대 변위를 계산하는 것이다. ICP와 같은 기존의 방법에서는 두 프레임 사이의 모든 pointcloud에서의 점을 사용하여 matching을 평가하지만, LOAM은 feature point를 사용하여 보다 압축적인 환경적 특징을 나타낸다. 
Curvature값에따라 feature point의 후보군을 정하고 다음의 규칙에 따라 최종적으로 feature point를 추출한다.

![LOAM figure 4](https://photos.app.goo.gl/G2bY6vutNTWxDnT29)

A point *i* can be selected only if
1. *S* does not form a surface patch that is roughly parallel to the laser beams
2. No point in *S* that is disconnected from *i* by a gap in the direction of the laser beam
3. No point in *S* that is at the same time closer to the LiDAR then point *i*
```c++
for (int i = 5; i < cloudSize - 6; i++) {
	//coplanar geometric relationship
    	float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    	float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    	float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    	float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;    	
    	//sqrt(diff) = 0.223
    	//두 포인트 사이 차이(곡률 크기)가 22cm 이상일 때 (curvature threshold filtering)
    	if (diff > 0.05) {      		
	      	float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x + 
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);
        	
        	float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x + 
                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);
		//For  fig4(b) in paper
		//point i가 point (i+1) 보다 뒤에 있을 때 (scan이 CW회전일경우)
        	if (depth1 > depth2) {
          		diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
          		diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
          		diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

        	  	if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
			//1일경우 Edge에서 제외        	  	
            		cloudNeighborPicked[i - 5] = 1;
            		cloudNeighborPicked[i - 4] = 1;
            		cloudNeighborPicked[i - 3] = 1;
            		cloudNeighborPicked[i - 2] = 1;
            		cloudNeighborPicked[i - 1] = 1;
            		cloudNeighborPicked[i] = 1;
          		}	
        	} 
        	//point (i+1)이 i보다 뒤에 있을 때 (scan이 CCW회전일경우)
        	else {
	          	diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
          		diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
          		diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

		        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
            		cloudNeighborPicked[i + 1] = 1;
            		cloudNeighborPicked[i + 2] = 1;
            		cloudNeighborPicked[i + 3] = 1;
            		cloudNeighborPicked[i + 4] = 1;
            		cloudNeighborPicked[i + 5] = 1;
            		cloudNeighborPicked[i + 6] = 1;
          		}
        	}
    	}
	// For fig4(a) in paper
    	float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    	float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    	float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    	float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    	float dis = laserCloud->points[i].x * laserCloud->points[i].x
              + laserCloud->points[i].y * laserCloud->points[i].y
              + laserCloud->points[i].z * laserCloud->points[i].z;

	if (diff > (0.25 * 0.25) / (20 * 20) * dis && diff2 > (0.25 * 0.25) / (20 * 20) * dis) {
		  cloudNeighborPicked[i] = 1;
    	}
}
```


**이 논문에서는 Feature point가 한곳에 몰리는 것을 방지하기 위해 한 scan line을 균등한 4개의 sub-region 나누고 한 sub-region 최대 2개의 edge point, 4개의 planar point를 추출할 수 있도록 제한 하였다.**

