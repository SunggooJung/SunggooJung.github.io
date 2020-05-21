---

title: "under construction"
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

- $S$ : 동일 scan내에서 얻어지는 특정 point *i*와 그 주변 point의 집합 (논문에서는 *i*의 앞뒤로 5개씩의 point를 선정)
    

#### 2. Feature Point Extraction

LOAM은 먼저 Feature Point를 추출하고 이 추출된 특징점들을 통해서 Odometry 추정과 mapping을 한다. 
이를 위해 2가지 종류의 Feature를 정의하고 있다. 

- Edge Point,  Planar Point

추출된 특징점이 Edge인지, Planar인지 구분하는 방식은 다음의 $c$(curvature)를 계산함으로서 구할 수 있다.

$\Large c = \frac{1}{|S|\cdot\Vert X^L_{(k,i)}\Vert}\Vert\sum_{j \in S, i \neq j}(X^L_{(k,i)}-X^L_{(k,j)})\Vert$ 

즉 검사하고자하는 point *i*의 주변 point와의 차이를 합하고 이를 


