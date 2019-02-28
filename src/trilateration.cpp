#include <ros/ros.h>
//#include <realtime_tools/realtime_publisher.h>
#include "trilateration.h"

#define		PI			3.1415926535

#define		array0_x	0
#define		array0_y	0
#define		array0_z	0.96

#define		array1_x	0
#define		array1_y	3
#define		array1_z	2

#define		array2_x	3
#define		array2_y	0
#define		array2_z	0.96

#define		array3_x	3
#define		array3_y	3
#define		array3_z	0.96

#define		TH			-55

vec3d vdiff(const vec3d vector1, const vec3d vector2)
{
	vec3d v;
	v.x = vector1.x - vector2.x;
	v.y = vector1.y - vector2.y;
	v.z = vector1.z - vector2.z;
	return v;
}

/* Return the sum of two vectors. */
vec3d vsum(const vec3d vector1, const vec3d vector2)
{
	vec3d v;
	v.x = vector1.x + vector2.x;
	v.y = vector1.y + vector2.y;
	v.z = vector1.z + vector2.z;
	return v;
}

/* Multiply vector by a number. */
vec3d vmul(const vec3d vector, const double n)
{
	vec3d v;
	v.x = vector.x * n;
	v.y = vector.y * n;
	v.z = vector.z * n;
	return v;
}

/* Divide vector by a number. */
vec3d vdiv(const vec3d vector, const double n)
{
	vec3d v;
	v.x = vector.x / n;
	v.y = vector.y / n;
	v.z = vector.z / n;
	return v;
}

/* Return the Euclidean norm. */
double vdist(const vec3d v1, const vec3d v2)
{
	double xd = v1.x - v2.x;
	double yd = v1.y - v2.y;
	double zd = v1.z - v2.z;
	return sqrt(xd * xd + yd * yd + zd * zd);
}

/* Return the Euclidean norm. */
double vnorm(const vec3d vector)
{
	return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

/* Return the dot product of two vectors. */
double dot(const vec3d vector1, const vec3d vector2)
{
	return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
}

/* Replace vector with its cross product with another vector. */
vec3d cross(const vec3d vector1, const vec3d vector2)
{
	vec3d v;
	v.x = vector1.y * vector2.z - vector1.z * vector2.y;
	v.y = vector1.z * vector2.x - vector1.x * vector2.z;
	v.z = vector1.x * vector2.y - vector1.y * vector2.x;
	return v;
}

/* Return the GDOP (Geometric Dilution of Precision) rate between 0-1.
 * Lower GDOP rate means better precision of intersection.
 */
double gdoprate(const vec3d tag, const vec3d p1, const vec3d p2, const vec3d p3)
{
	vec3d ex, t1, t2, t3;
	double h, gdop1, gdop2, gdop3, result;

	ex = vdiff(p1, tag);
	h = vnorm(ex);
	t1 = vdiv(ex, h);

	ex = vdiff(p2, tag);
	h = vnorm(ex);
	t2 = vdiv(ex, h);

	ex = vdiff(p3, tag);
	h = vnorm(ex);
	t3 = vdiv(ex, h);

	gdop1 = fabs(dot(t1, t2));
	gdop2 = fabs(dot(t2, t3));
	gdop3 = fabs(dot(t3, t1));

	if (gdop1 < gdop2) result = gdop2; else result = gdop1;
	if (result < gdop3) result = gdop3;

	return result;
}

int sphereline(const vec3d p1, const vec3d p2, const vec3d sc, double r, double *const mu1, double *const mu2)
{
   double a,b,c;
   double bb4ac;
   vec3d dp;

   dp.x = p2.x - p1.x;
   dp.y = p2.y - p1.y;
   dp.z = p2.z - p1.z;

   a = dp.x * dp.x + dp.y * dp.y + dp.z * dp.z;

   b = 2 * (dp.x * (p1.x - sc.x) + dp.y * (p1.y - sc.y) + dp.z * (p1.z - sc.z));

   c = sc.x * sc.x + sc.y * sc.y + sc.z * sc.z;
   c += p1.x * p1.x + p1.y * p1.y + p1.z * p1.z;
   c -= 2 * (sc.x * p1.x + sc.y * p1.y + sc.z * p1.z);
   c -= r * r;

   bb4ac = b * b - 4 * a * c;

   if (fabs(a) == 0 || bb4ac < 0) {
      *mu1 = 0;
      *mu2 = 0;
      return -1;
   }

   *mu1 = (-b + sqrt(bb4ac)) / (2 * a);
   *mu2 = (-b - sqrt(bb4ac)) / (2 * a);

   return 0;
}

int _trilateration(vec3d *const result1,
				  vec3d *const result2,
				  vec3d *const best_solution,
				  const vec3d p1, const double r1,
                  const vec3d p2, const double r2,
                  const vec3d p3, const double r3,
                  const vec3d p4, const double r4,
                  const double maxzero)
{
	vec3d	ex, ey, ez, t1, t2, t3;
	double	h, i, j, x, y, z, t;
	double	mu1, mu2, mu;
	int result;

	/*********** FINDING TWO POINTS FROM THE FIRST THREE SPHERES **********/

	// if there are at least 2 concentric spheres within the first 3 spheres
	// then the calculation may not continue, drop it with error -1

	/* h = |p3 - p1|, ex = (p3 - p1) / |p3 - p1| */
	ex = vdiff(p3, p1); // vector p13
	h = vnorm(ex); // scalar p13
	if (h <= maxzero) {
		/* p1 and p3 are concentric, not good to obtain a precise intersection point */
		//printf("concentric13 return -1\n");
		return ERR_TRIL_CONCENTRIC;
	}

	/* h = |p3 - p2|, ex = (p3 - p2) / |p3 - p2| */
	ex = vdiff(p3, p2); // vector p23
	h = vnorm(ex); // scalar p23
	if (h <= maxzero) {
		/* p2 and p3 are concentric, not good to obtain a precise intersection point */
		//printf("concentric23 return -1\n");
		return ERR_TRIL_CONCENTRIC;
	}

	/* h = |p2 - p1|, ex = (p2 - p1) / |p2 - p1| */
	ex = vdiff(p2, p1); // vector p12
	h = vnorm(ex); // scalar p12
	if (h <= maxzero) {
		/* p1 and p2 are concentric, not good to obtain a precise intersection point */
		//printf("concentric12 return -1\n");
		return ERR_TRIL_CONCENTRIC;
	}
	ex = vdiv(ex, h); // unit vector ex with respect to p1 (new coordinate system)

	/* t1 = p3 - p1, t2 = ex (ex . (p3 - p1)) */
	t1 = vdiff(p3, p1); // vector p13
	i = dot(ex, t1); // the scalar of t1 on the ex direction
	t2 = vmul(ex, i); // colinear vector to p13 with the length of i

	/* ey = (t1 - t2), t = |t1 - t2| */
	ey = vdiff(t1, t2); // vector t21 perpendicular to t1
	t = vnorm(ey); // scalar t21
	if (t > maxzero) {
		/* ey = (t1 - t2) / |t1 - t2| */
		ey = vdiv(ey, t); // unit vector ey with respect to p1 (new coordinate system)

		/* j = ey . (p3 - p1) */
		j = dot(ey, t1); // scalar t1 on the ey direction
	} else
		j = 0.0;

	/* Note: t <= maxzero implies j = 0.0. */
	if (fabs(j) <= maxzero) {

		/* Is point p1 + (r1 along the axis) the intersection? */
		t2 = vsum(p1, vmul(ex, r1));
		if (fabs(vnorm(vdiff(p2, t2)) - r2) <= maxzero &&
		    fabs(vnorm(vdiff(p3, t2)) - r3) <= maxzero) {
			/* Yes, t2 is the only intersection point. */
			if (result1)
				*result1 = t2;
			if (result2)
				*result2 = t2;
			return TRIL_3SPHERES;
		}

		/* Is point p1 - (r1 along the axis) the intersection? */
		t2 = vsum(p1, vmul(ex, -r1));
		if (fabs(vnorm(vdiff(p2, t2)) - r2) <= maxzero &&
		    fabs(vnorm(vdiff(p3, t2)) - r3) <= maxzero) {
			/* Yes, t2 is the only intersection point. */
			if (result1)
				*result1 = t2;
			if (result2)
				*result2 = t2;
			return TRIL_3SPHERES;
		}
		/* p1, p2 and p3 are colinear with more than one solution */
		return ERR_TRIL_COLINEAR_2SOLUTIONS;
	}

	/* ez = ex x ey */
	ez = cross(ex, ey); // unit vector ez with respect to p1 (new coordinate system)

	x = (r1*r1 - r2*r2) / (2*h) + h / 2;
	y = (r1*r1 - r3*r3 + i*i) / (2*j) + j / 2 - x * i / j;
	z = r1*r1 - x*x - y*y;
	if (z < -maxzero) {
		/* The solution is invalid, square root of negative number */
		return ERR_TRIL_SQRTNEGNUMB;
	} else
	if (z > 0.0)
		z = sqrt(z);
	else
		z = 0.0;

	/* t2 = p1 + x ex + y ey */
	t2 = vsum(p1, vmul(ex, x));
	t2 = vsum(t2, vmul(ey, y));

	/* result1 = p1 + x ex + y ey + z ez */
	if (result1)
		*result1 = vsum(t2, vmul(ez, z));

	/* result1 = p1 + x ex + y ey - z ez */
	if (result2)
		*result2 = vsum(t2, vmul(ez, -z));

	/*********** END OF FINDING TWO POINTS FROM THE FIRST THREE SPHERES **********/
	/********* RESULT1 AND RESULT2 ARE SOLUTIONS, OTHERWISE RETURN ERROR *********/


	/************* FINDING ONE SOLUTION BY INTRODUCING ONE MORE SPHERE ***********/

	// check for concentricness of sphere 4 to sphere 1, 2 and 3
	// if it is concentric to one of them, then sphere 4 cannot be used
	// to determine the best solution and return -1

	/* h = |p4 - p1|, ex = (p4 - p1) / |p4 - p1| */
	ex = vdiff(p4, p1); // vector p14
	h = vnorm(ex); // scalar p14
	if (h <= maxzero) {
		/* p1 and p4 are concentric, not good to obtain a precise intersection point */
		//printf("concentric14 return 0\n");
		return TRIL_3SPHERES;
	}
	/* h = |p4 - p2|, ex = (p4 - p2) / |p4 - p2| */
	ex = vdiff(p4, p2); // vector p24
	h = vnorm(ex); // scalar p24
	if (h <= maxzero) {
		/* p2 and p4 are concentric, not good to obtain a precise intersection point */
		//printf("concentric24 return 0\n");
		return TRIL_3SPHERES;
	}
	/* h = |p4 - p3|, ex = (p4 - p3) / |p4 - p3| */
	ex = vdiff(p4, p3); // vector p34
	h = vnorm(ex); // scalar p34
	if (h <= maxzero) {
		/* p3 and p4 are concentric, not good to obtain a precise intersection point */
		//printf("concentric34 return 0\n");
		return TRIL_3SPHERES;
	}

	// if sphere 4 is not concentric to any sphere, then best solution can be obtained
	/* find i as the distance of result1 to p4 */
	t3 = vdiff(*result1, p4);
	i = vnorm(t3);
	/* find h as the distance of result2 to p4 */
	t3 = vdiff(*result2, p4);
	h = vnorm(t3);

	/* pick the result1 as the nearest point to the center of sphere 4 */
	if (i > h) {
		*best_solution = *result1;
		*result1 = *result2;
		*result2 = *best_solution;
	}

	int count4 = 0;
	double rr4 = r4;
	result = 1;
	/* intersect result1-result2 vector with sphere 4 */
	while(result && count4 < 10)
	{
		result=sphereline(*result1, *result2, p4, rr4, &mu1, &mu2);
		rr4+=0.1;
		count4++;
	}

	if (result) {

		/* No intersection between sphere 4 and the line with the gradient of result1-result2! */
		*best_solution = *result1; // result1 is the closer solution to sphere 4
		//return ERR_TRIL_NOINTERSECTION_SPHERE4;

	} else {

		if (mu1 < 0 && mu2 < 0) {

			/* if both mu1 and mu2 are less than 0 */
			/* result1-result2 line segment is outside sphere 4 with no intersection */
			if (fabs(mu1) <= fabs(mu2)) mu = mu1; else mu = mu2;
			/* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
			ex = vdiff(*result2, *result1); // vector result1-result2
			h = vnorm(ex); // scalar result1-result2
			ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
			/* 50-50 error correction for mu */
			mu = 0.5*mu;
			/* t2 points to the intersection */
			t2 = vmul(ex, mu*h);
			t2 = vsum(*result1, t2);
			/* the best solution = t2 */
			*best_solution = t2;

		} else if ((mu1 < 0 && mu2 > 1) || (mu2 < 0 && mu1 > 1)) {

			/* if mu1 is less than zero and mu2 is greater than 1, or the other way around */
			/* result1-result2 line segment is inside sphere 4 with no intersection */
			if (mu1 > mu2) mu = mu1; else mu = mu2;
			/* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
			ex = vdiff(*result2, *result1); // vector result1-result2
			h = vnorm(ex); // scalar result1-result2
			ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
			/* t2 points to the intersection */
			t2 = vmul(ex, mu*h);
			t2 = vsum(*result1, t2);
			/* vector t2-result2 with 50-50 error correction on the length of t3 */
			t3 = vmul(vdiff(*result2, t2),0.5);
			/* the best solution = t2 + t3 */
			*best_solution = vsum(t2, t3);

		} else if (((mu1 > 0 && mu1 < 1) && (mu2 < 0 || mu2 > 1))
				|| ((mu2 > 0 && mu2 < 1) && (mu1 < 0 || mu1 > 1))) {

			/* if one mu is between 0 to 1 and the other is not */
			/* result1-result2 line segment intersects sphere 4 at one point */
			if (mu1 >= 0 && mu1 <= 1) mu = mu1; else mu = mu2;
			/* add or subtract with 0.5*mu to distribute error equally onto every sphere */
			if (mu <= 0.5) mu-=0.5*mu; else mu-=0.5*(1-mu);
			/* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
			ex = vdiff(*result2, *result1); // vector result1-result2
			h = vnorm(ex); // scalar result1-result2
			ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
			/* t2 points to the intersection */
			t2 = vmul(ex, mu*h);
			t2 = vsum(*result1, t2);
			/* the best solution = t2 */
			*best_solution = t2;

		} else if (mu1 == mu2) {

			/* if both mu1 and mu2 are between 0 and 1, and mu1 = mu2 */
			/* result1-result2 line segment is tangential to sphere 4 at one point */
			mu = mu1;
			/* add or subtract with 0.5*mu to distribute error equally onto every sphere */
			if (mu <= 0.25) mu-=0.5*mu;
			else if (mu <=0.5) mu-=0.5*(0.5-mu);
			else if (mu <=0.75) mu-=0.5*(mu-0.5);
			else mu-=0.5*(1-mu);
			/* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
			ex = vdiff(*result2, *result1); // vector result1-result2
			h = vnorm(ex); // scalar result1-result2
			ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
			/* t2 points to the intersection */
			t2 = vmul(ex, mu*h);
			t2 = vsum(*result1, t2);
			/* the best solution = t2 */
			*best_solution = t2;

		} else {

			/* if both mu1 and mu2 are between 0 and 1 */
			/* result1-result2 line segment intersects sphere 4 at two points */

			//return ERR_TRIL_NEEDMORESPHERE;

			mu = mu1 + mu2;
			/* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
			ex = vdiff(*result2, *result1); // vector result1-result2
			h = vnorm(ex); // scalar result1-result2
			ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
			/* 50-50 error correction for mu */
			mu = 0.5*mu;
			/* t2 points to the intersection */
			t2 = vmul(ex, mu*h);
			t2 = vsum(*result1, t2);
			/* the best solution = t2 */
			*best_solution = t2;

		}

	}

	return TRIL_4SPHERES;

	/******** END OF FINDING ONE SOLUTION BY INTRODUCING ONE MORE SPHERE *********/
}
int deca_3dlocate (	vec3d	*const solution1,
					vec3d	*const solution2,
					vec3d	*const best_solution,
					int		*const nosolution_count,
					double	*const best_3derror,
					double	*const best_gdoprate,
					vec3d p1, double r1,
					vec3d p2, double r2,
					vec3d p3, double r3,
					vec3d p4, double r4,
					int *combination)
{
	vec3d	o1, o2, solution, ptemp;
	vec3d	solution_compare1, solution_compare2;
    double	/*error_3dcompare1, error_3dcompare2,*/ rtemp;
	double	gdoprate_compare1, gdoprate_compare2;
	double	ovr_r1, ovr_r2, ovr_r3, ovr_r4;
	int		overlook_count, combination_counter;
	int		trilateration_errcounter, trilateration_mode34;
	int		success, concentric, result;

	trilateration_errcounter = 0;
	trilateration_mode34 = 0;

	combination_counter = 4; /* four spheres combination */

	*best_gdoprate = 1; /* put the worst gdoprate init */
	gdoprate_compare1 = 1; gdoprate_compare2 = 1;
	solution_compare1.x = 0; solution_compare1.y = 0; solution_compare1.z = 0;
    //error_3dcompare1 = 0;

	do {
		success = 0;
		concentric = 0;
		overlook_count = 0;
		ovr_r1 = r1; ovr_r2 = r2; ovr_r3 = r3; ovr_r4 = r4;

		do {

			result = _trilateration(&o1, &o2, &solution, p1, ovr_r1, p2, ovr_r2, p3, ovr_r3, p4, ovr_r4, MAXZERO);

			switch (result)
			{
				case TRIL_3SPHERES: // 3 spheres are used to get the result
					trilateration_mode34 = TRIL_3SPHERES;
					success = 1;
					break;

				case TRIL_4SPHERES: // 4 spheres are used to get the result
					trilateration_mode34 = TRIL_4SPHERES;
					success = 1;
					break;

				case ERR_TRIL_CONCENTRIC:
					concentric = 1;
					break;

				default: // any other return value goes here
					ovr_r1 += 0.10;
					ovr_r2 += 0.10;
					ovr_r3 += 0.10;
					ovr_r4 += 0.10;
					overlook_count++;
					break;
			}

            //qDebug() << "while(!success)" << overlook_count << concentric << "result" << result;

        } while (!success && (overlook_count <= 5) && !concentric);


		if (success)
		{
			switch (result)
			{
			case TRIL_3SPHERES:
				*solution1 = o1;
				*solution2 = o2;
				*nosolution_count = overlook_count;

                combination_counter = 0;
				break;

			case TRIL_4SPHERES:
				/* calculate the new gdop */
				gdoprate_compare1	= gdoprate(solution, p1, p2, p3);

				/* compare and swap with the better result */
				if (gdoprate_compare1 <= gdoprate_compare2) {

					*solution1 = o1;
					*solution2 = o2;
					*best_solution	= solution;
					*nosolution_count = overlook_count;
					*best_3derror	= sqrt((vnorm(vdiff(solution, p1))-r1)*(vnorm(vdiff(solution, p1))-r1) +
										(vnorm(vdiff(solution, p2))-r2)*(vnorm(vdiff(solution, p2))-r2) +
										(vnorm(vdiff(solution, p3))-r3)*(vnorm(vdiff(solution, p3))-r3) +
										(vnorm(vdiff(solution, p4))-r4)*(vnorm(vdiff(solution, p4))-r4));
					*best_gdoprate	= gdoprate_compare1;

					/* save the previous result */
					solution_compare2 = solution_compare1;
                    //error_3dcompare2 = error_3dcompare1;
					gdoprate_compare2 = gdoprate_compare1;

					*combination = 5 - combination_counter;

                    ptemp = p1; p1 = p2; p2 = p3; p3 = p4; p4 = ptemp;
                    rtemp = r1; r1 = r2; r2 = r3; r3 = r4; r4 = rtemp;
                    combination_counter--;

				}
				break;

			default:
				break;
			}
		}
		else
		{
            //trilateration_errcounter++;
            trilateration_errcounter = 4;
            combination_counter = 0;
		}

        //ptemp = p1; p1 = p2; p2 = p3; p3 = p4; p4 = ptemp;
        //rtemp = r1; r1 = r2; r2 = r3; r3 = r4; r4 = rtemp;
        //combination_counter--;
        //qDebug() << "while(combination_counter)" << combination_counter;

    } while (combination_counter);

	// if it gives error for all 4 sphere combinations then no valid result is given
	// otherwise return the trilateration mode used
	if (trilateration_errcounter >= 4) return -1; else return trilateration_mode34;

}


int GetLocation(vec3d *best_solution, int use4thAnchor, vec3d* anchorArray, int *distanceArray)
{

	vec3d	o1, o2, p1, p2, p3, p4;
    double	r1 = 0, r2 = 0, r3 = 0, r4 = 0, best_3derror, best_gdoprate;
	int		result;
    int     error, combination;

	vec3d	t3;
	double	dist1, dist2;

	/* Anchors coordinate */
    p1.x = anchorArray[0].x;		p1.y = anchorArray[0].y;	p1.z = anchorArray[0].z;
    p2.x = anchorArray[1].x;		p2.y = anchorArray[1].y;	p2.z = anchorArray[1].z;
    p3.x = anchorArray[2].x;		p3.y = anchorArray[2].y;	p3.z = anchorArray[2].z;
    p4.x = anchorArray[0].x;		p4.y = anchorArray[0].y;	p4.z = anchorArray[0].z; //4th same as 1st - only 3 used for trilateration

    r1 = (double) distanceArray[0] / 1000.0;
    r2 = (double) distanceArray[1] / 1000.0;
    r3 = (double) distanceArray[2] / 1000.0;

    r4 = (double) distanceArray[3] / 1000.0;

    //qDebug() << "GetLocation" << r1 << r2 << r3 << r4;

    //r4 = r1;

	/* get the best location using 3 or 4 spheres and keep it as know_best_location */
    result = deca_3dlocate (&o1, &o2, best_solution, &error, &best_3derror, &best_gdoprate,
                            p1, r1, p2, r2, p3, r3, p4, r1, &combination);


    //qDebug() << "GetLocation" << result << "sol1: " << o1.x << o1.y << o1.z << " sol2: " << o2.x << o2.y << o2.z;

    if(result >= 0)
    {
        if (use4thAnchor == 1) //if have 4 ranging results, then use 4th anchor to pick solution closest to it
        {
                double diff1, diff2;
                dist1 = sqrt((o1.x-array0_x)*(o1.x-array0_x)+(o1.y-array0_y)*(o1.y-array0_y)+(o1.z-array0_z)*(o1.z-array0_z));
		dist2 = sqrt((o2.x-array0_x)*(o2.x-array0_x)+(o2.y-array0_y)*(o2.y-array0_y)+(o2.z-array0_z)*(o2.z-array0_z));
		diff1 = fabs(r4 - dist1);
                diff2 = fabs(r4 - dist2);
		ROS_INFO_STREAM("sol1 : " << o1.x <<" "<< o1.y <<" "<< o1.z << " sol2: " << o2.x <<" "<< o2.y <<" "<< o2.z);

                /* pick the closest match to the 4th anchor range */
                if (diff1 < diff2){
			*best_solution = o1;
			ROS_INFO_STREAM("sol1");
		}
		else {
			*best_solution = o2;
			ROS_INFO_STREAM("sol2");
		}
        }
        else
        {
            //assume tag is below the anchors (1, 2, and 3)
            if(o1.z < p1.z) *best_solution = o1; else *best_solution = o2;
        }
    }

	if (result >= 0)
	{
		return result;
	}

	//return error
	return -1;
}

int calculateTagLocation(vec3d *report, int count, int *ranges)
{
    int result = 0;
    vec3d anchorArray[4];
	//앵커 초기 좌표 m단위
	anchorArray[0].x = array0_x;//_ancArray[0].x;
    anchorArray[0].y = array0_y;//_ancArray[0].y;
    anchorArray[0].z = array0_z;//_ancArray[0].z;
//2
    anchorArray[1].x = array1_x;//_ancArray[1].x;
    anchorArray[1].y = array1_y;//_ancArray[1].y;
    anchorArray[1].z = array1_z;//_ancArray[1].z;
//
    anchorArray[2].x = array2_x;//_ancArray[2].x;
    anchorArray[2].y = array2_y;//_ancArray[2].y;
    anchorArray[2].z = array2_z;//_ancArray[2].z;
//
    anchorArray[3].x = array3_x;//_ancArray[3].x;
    anchorArray[3].y = array3_y;//_ancArray[3].y;
    anchorArray[3].z = array3_z;//_ancArray[3].z;

    result = GetLocation(report, ((count==4) ? 1 : 0), &anchorArray[0], ranges);

    return result;
}
void RotationTag(vec3d *report, double theta){
	vec3d result;

	result.x = report->x * cos((PI*theta)/180.0) - report->y * sin((PI*theta)/180.0);
	result.y = report->x * sin((PI*theta)/180.0) + report->y * cos((PI*theta)/180.0);
	report->x = result.x;
	report->y = result.y;
	
}
void orientationCB(const geometry_msgs::PoseStamped msg)
{
	/*
	tag_xyz.pose.orientation.x = msg.pose.orientation.x;
	tag_xyz.pose.orientation.y = msg.pose.orientation.y;
	tag_xyz.pose.orientation.z = msg.pose.orientation.z;
	tag_xyz.pose.orientation.w = msg.pose.orientation.w;*/
	tag_xyz.pose.orientation.x = 0;
	tag_xyz.pose.orientation.y = 0;
	tag_xyz.pose.orientation.z = 0;
	tag_xyz.pose.orientation.w = 1;
}
void taglocationCB(const geometry_msgs::PoseStamped msg)
{
	//tag가 보내주는 앵커간의 거리를 배열에 저장함
	ranges[0] = (double)msg.pose.orientation.x;
	ranges[1] = (double)msg.pose.orientation.y;
	ranges[2] = (double)msg.pose.orientation.z;
	ranges[3] = (double)msg.pose.orientation.w;
	calculateTagLocation(&report, 4, ranges);
	//RotationTag(&report, TH);
	tag_xyz.pose.position.x = report.x;
	tag_xyz.pose.position.y = report.y;
	tag_xyz.pose.position.z = report.z;
	//tag_xyz.pose.position.z = 1;

}

int main(int argc, char **argv)
{
	//여기서 함수 호출.
	//4변값을 받아서 xyz값을 topic으로 던져줌
	ros::init(argc, argv, "trilateration_node");
	ros::NodeHandle nh;
	ros::Rate rate(7);
	
	//int ranges[4]={ 707, 707, 707, 1224 };//500,500,0 이 나와야함 단위는 mm
	//int ranges[4]={ 250, 750, 1030, 1600 };//250,0,0 이 나와야함 단위는 mm
	//int ranges[4]={ 768, 768, 768, 994 };//500,500,300 이 나와야함 단위는 mm
	//int ranges[4]={ 768, 768, 768, 994 };//앵커간 거리 mm단위
	ros::Publisher tag_xyz_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
	//realtime_tagxyz_pub.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(nh, "/mavros/vision_pose/pose", 4));
	//xyz publish하는 객체
	ros::Subscriber anchor_distance_sub = nh.subscribe("/anchorD", 10, taglocationCB);
	ros::Subscriber orientation_sub = nh.subscribe("/mavros/local_position/pose", 10, orientationCB);
	int count = 0;
	while(ros::ok()){
		//calculateTagLocation(&report, 4, ranges);
		ROS_INFO_STREAM("x : " << report.x << "  y : " << report.y << "  z : " << report.z );
		tag_xyz.pose.orientation.x = 0;
		tag_xyz.pose.orientation.y = 0;
		tag_xyz.pose.orientation.z = 0;
		tag_xyz.pose.orientation.w = 1;
		/*
		if(realtime_tagxyz_pub->trylock()){
			realtime_tagxyz_pub->msg_.header.stamp = ros::Time::now();
			realtime_tagxyz_pub->msg_.header.seq = count;
			realtime_tagxyz_pub->msg_.pose.position.x = report.x;
			realtime_tagxyz_pub->msg_.pose.position.y = report.y;
			realtime_tagxyz_pub->msg_.pose.position.z = report.z;
			realtime_tagxyz_pub->msg_.pose.orientation.x = 0;
			realtime_tagxyz_pub->msg_.pose.orientation.y = 0;
			realtime_tagxyz_pub->msg_.pose.orientation.z = 0;
			realtime_tagxyz_pub->msg_.pose.orientation.w = 1;
			realtime_tagxyz_pub->unlockAndPublish();
			count++;
		}*/
		tag_xyz.header.stamp = ros::Time::now();
		tag_xyz.header.seq = count;
		tag_xyz_pub.publish(tag_xyz);
		count ++;
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}

