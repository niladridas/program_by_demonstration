/*
 * circlefit3d.cpp
 *
 *  Created on: May 27, 2014
 *      Author: niladri-64
 */
#include<stdio.h>
#include<math.h>
#include<iostream>


 /* Function Definitions */
   /* Function Definitions */
void circlefit3d(double p1[3], double p2[3], double p3[3],
                 double center_data[3], double rad_data[1])
{
	double v2[3];
  int i0;
  double dotp_data_idx_0;
  double v1[3];
  double l1;
  double l2;
  int i;
  double p3_2d[2];


  int center_size[2];
  int rad_size[1];
  double v1n_data[3];
  int v1n_size[2];
  double v2nb_data[3];
  int v2nb_size[2];

  /*  circlefit3d: Compute center and radii of circles in 3d which are defined by three points on the circumference */
  /*  usage: [center,rad,v1,v2] = circlefit3d(p1,p2,p3) */
  /*  */
  /*  arguments: (input) */
  /*   p1, p2, p3 - vectors of points (rowwise, size(p1) = [n 3]) */
  /*                describing the three corresponding points on the same circle. */
  /*                p1, p2 and p3 must have the same length n. */
  /*  */
  /*  arguments: (output) */
  /*   center - (nx3) matrix of center points for each triple of points in p1,  p2, p3 */
  /*  */
  /*   rad    - (nx1) vector of circle radii. */
  /*            if there have been errors, radii is a negative scalar ( = error code) */
  /*  */
  /*   v1, v2 - (nx3) perpendicular vectors inside circle plane */
  /*  */
  /*  Example usage: */
  /*  */
  /*   (1) */
  /*       p1 = rand(10,3); */
  /*       p2 = rand(10,3); */
  /*       p3 = rand(10,3); */
  /*       [center, rad] = circlefit3d(p1,p2,p3); */
  /*       % verification, result should be all (nearly) zero */
  /*       result(:,1)=sqrt(sum((p1-center).^2,2))-rad; */
  /*       result(:,2)=sqrt(sum((p2-center).^2,2))-rad; */
  /*       result(:,3)=sqrt(sum((p3-center).^2,2))-rad; */
  /*       if sum(sum(abs(result))) < 1e-12, */
  /*        disp('All circles have been found correctly.'); */
  /*       else, */
  /*        disp('There had been errors.'); */
  /*       end */
  /*  */
  /*  */
  /*  (2) */
  /*        p1=rand(4,3);p2=rand(4,3);p3=rand(4,3); */
  /*        [center,rad,v1,v2] = circlefit3d(p1,p2,p3); */
  /*        plot3(p1(:,1),p1(:,2),p1(:,3),'bo');hold on;plot3(p2(:,1),p2(:,2),p2(:,3),'bo');plot3(p3(:,1),p3(:,2),p3(:,3),'bo'); */
  /*        for i=1:361, */
  /*            a = i/180*pi; */
  /*            x = center(:,1)+sin(a)*rad.*v1(:,1)+cos(a)*rad.*v2(:,1); */
  /*            y = center(:,2)+sin(a)*rad.*v1(:,2)+cos(a)*rad.*v2(:,2); */
  /*            z = center(:,3)+sin(a)*rad.*v1(:,3)+cos(a)*rad.*v2(:,3); */
  /*            plot3(x,y,z,'r.'); */
  /*        end */
  /*        axis equal;grid on;rotate3d on; */
  /*  */
  /*   */
  /*  Author: Johannes Korsawe */
  /*  E-mail: johannes.korsawe@volkswagen.de */
  /*  Release: 1.0 */
  /*  Release date: 26/01/2012 */
  /*  Default values */
  /*  check inputs */
  /*  check number of inputs */
  /*  check size of inputs */
  /*  more checks are to follow inside calculation */
  /*  Start calculation */
  /*  v1, v2 describe the vectors from p1 to p2 and p3, resp. */
  /*  l1, l2 describe the lengths of those vectors */
  for (i0 = 0; i0 < 3; i0++) {
    dotp_data_idx_0 = p2[i0] - p1[i0];
    v2[i0] = p3[i0] - p1[i0];
    v1n_data[i0] = dotp_data_idx_0;
    v1[i0] = dotp_data_idx_0;
  }

  l1 = sqrt((v1[0] * v1[0] + v1[1] * v1[1]) + v1[2] * v1[2]);
  l2 = sqrt((v2[0] * v2[0] + v2[1] * v2[1]) + v2[2] * v2[2]);

  /*  if find(l1==0) | find(l2==0), %#ok<OR2> */
  /*      fprintf('??? Error using ==> cirlefit3d\nCorresponding input points must not be identical.\n');rad = -4;return; */
  /*  end */
  /*  v1n, v2n describe the normalized vectors v1 and v2 */
  v1n_size[0] = 1;
  v1n_size[1] = 3;
  for (i = 0; i < 3; i++) {
    v1n_data[i] /= l1;
  }

  /*  nv describes the normal vector on the plane of the circle */
  /*  if find(sum(abs(nv),2)<1e-5), */
  /*      fprintf('??? Warning using ==> cirlefit3d\nSome corresponding input points are nearly collinear.\n'); */
  /*  end */
  /*  v2nb: orthogonalization of v2n against v1n */
  for (i0 = 0; i0 < 3; i0++) {
    dotp_data_idx_0 = v2[i0] / l2;
    v2nb_data[i0] = dotp_data_idx_0;
    v1[i0] = dotp_data_idx_0;
  }

  dotp_data_idx_0 = (v1[0] * v1n_data[0] + v1[1] * v1n_data[1]) + v1[2] *
    v1n_data[2];
  v2nb_size[0] = 1;
  v2nb_size[1] = 3;
  for (i = 0; i < 3; i++) {
    v2nb_data[i] -= dotp_data_idx_0 * v1n_data[i];
  }

  /*  normalize v2nb */
  dotp_data_idx_0 = sqrt((v2nb_data[0] * v2nb_data[0] + v2nb_data[1] *
    v2nb_data[1]) + v2nb_data[2] * v2nb_data[2]);
  for (i = 0; i < 3; i++) {
    v2nb_data[i] /= dotp_data_idx_0;
  }

  /*  remark: the circle plane will now be discretized as follows */
  /*  */
  /*  origin: p1                    normal vector on plane: nv */
  /*  first coordinate vector: v1n  second coordinate vector: v2nb */
  /*  calculate 2d coordinates of points in each plane */
  /*  p1_2d = zeros(n,2); % set per construction */
  /*  p2_2d = zeros(n,2);p2_2d(:,1) = l1; % set per construction */
  for (i0 = 0; i0 < 2; i0++) {
    p3_2d[i0] = 0.0;
  }

  /*  has to be calculated */
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      p3_2d[i0] += v2[i0 + i] * v1n_data[i0 + i];
      p3_2d[1 + i0] += v2[i0 + i] * v2nb_data[i0 + i];
    }
  }

  /*  calculate the fitting circle  */
  /*  due to the special construction of the 2d system this boils down to solving */
  /*  q1 = [0,0], q2 = [a,0], q3 = [b,c] (points on 2d circle) */
  /*  crossing perpendicular bisectors, s and t running indices: */
  /*  solve [a/2,s] = [b/2 + c*t, c/2 - b*t] */
  /*  solution t = (a-b)/(2*c) */
  l2 = 0.5 * (l1 - p3_2d[0]) / p3_2d[1];
  dotp_data_idx_0 = p3_2d[0] / 2.0 + p3_2d[1] * l2;
  l2 = p3_2d[1] / 2.0 - p3_2d[0] * l2;

  /*  centers */
  center_size[0] = 1;
  center_size[1] = 3;
  for (i0 = 0; i0 < 3; i0++) {
    center_data[i0] = 0.0;
  }

  for (i = 0; i < 3; i++) {
    center_data[i] = (p1[i] + dotp_data_idx_0 * v1n_data[i]) + l2 * v2nb_data[i];
  }

  /*  radii */
  dotp_data_idx_0 = center_data[0] - p1[0];
  rad_data[0] = dotp_data_idx_0 * dotp_data_idx_0;
  dotp_data_idx_0 = center_data[1] - p1[1];
  l2 = dotp_data_idx_0 * dotp_data_idx_0;
  dotp_data_idx_0 = center_data[2] - p1[2];
  rad_size[0] = 1;
  rad_data[0] = (rad_data[0] + l2) + dotp_data_idx_0 * dotp_data_idx_0;
  rad_data[0] = sqrt(rad_data[0]);

}

/* End of code generation (circlefit3d.cpp) */
int main()
{

double p1[3] = {0.0895, -0.2981, 1.0974};
double p2[3] = {-0.0922, -0.1125, 0.9499};
double p3[3] = {-0.3840, -0.0175, 0.9564};
double center_data[3] = {0,0,0};
double rd_data[1] = {0};

circlefit3d(p1,p2,p3,center_data,rd_data);
std::cout << rd_data[0] << std::endl;


	return 0;
}
