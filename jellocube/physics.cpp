/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"
#include <iostream>

using namespace std;


/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */

/*  To compute the acceleration, the function must take into account the forces due to
1)structural, shear and bend springs,
2)external forces (force field, if any), and
3)bouncing off the walls.*/

// Calculate the dot product of two points 
double dotProduct(point A,point B)
{
	double res;
	res = A.x*B.x + A.y*B.y + A.z*B.z;
	return res;
}

// Calculate spring force and damping force
void springForce(point L, point V, point* f, double R, double ks, double kd)
{
  double length, temp;
  point F;
	point Fh = { 0, 0, 0 }, Fd = { 0, 0, 0 };
	length = sqrt(pow((L).x,2)+pow((L).y,2)+pow((L).z,2));
  memset(f, 0, sizeof(point));

	if (length != 0) 
  {
	  //Hook
	  pMULTIPLY(L, -ks * (length - R) / length, Fh);
	  //Damping
	  temp = dotProduct(V, L);
	  pMULTIPLY(L, -kd * temp / length / length, Fd);
	  pSUM(Fh, Fd, *f);
  } 
  else return;
}

// Add spring force and damping force together for each mass point
void computeSpringForce(world* jello, int i, int j, int k, point* F)
{	
  // Fh = k*(|L|-R)*(L/|L|)
  // Fd = - kd*((Va-Vb)L)/|L|*(L/|L|)

  double ks = jello -> kElastic;
  double kd = jello -> dElastic;
  double R = 0;
  double Slength = 0.0;
  point L={0, 0, 0}, V={0, 0, 0},f = { 0, 0, 0 };
  double tempt;
  point Fstruct, Fshear, Fbend;

	memset(&Fstruct, 0, sizeof(point));
	memset(&Fshear, 0, sizeof(point));
	memset(&Fbend, 0, sizeof(point));

	for (int l = -1; l < 2; l++)
  {
		for (int m = -1; m < 2; m++)
    {
      for (int n = -1; n < 2; n++)
			{
        R = sqrt(abs(l) + abs(m) + abs(n));
				if (i + l < 0 || i + l >7 || j + m < 0 || j + m > 7 || k + n < 0 || k + n > 7|| R == 0)
				continue;
        // Calculate vector L, V
				pDIFFERENCE(jello->p[i][j][k], jello->p[i + l][j + m][k + n], L);
        pDIFFERENCE(jello->v[i][j][k], jello->v[i + l][j + m][k + n], V);

        springForce(L, V, &f, 1.0 / 7*R, ks, kd);
        if (R > 1)
        {//shear spring force
					pSUM(Fshear, f, Fshear);
				}
				else
        {//structural spring force
					pSUM(Fstruct, f, Fstruct);
				}						            
			}							
		}
	}
  // Bend spring
  R = 2;
  #define BENDFORCE(di,dj,dk) \
	if(!(i + di < 0 || i + di >7 || j + dj < 0 || j + dj > 7 || k + dk < 0 || k + dk > 7)){\
	pDIFFERENCE(jello->v[i][j][k], jello->v[i+di][j+dj][k+dk], V);\
	pDIFFERENCE(jello->p[i][j][k], jello->p[i+di][j+dj][k+dk], L);\
  springForce(L,V,&f, 1.0 / 7*R,ks,kd);\
  pSUM(Fbend, f, Fbend);\
	}
  BENDFORCE(2, 0, 0);
	BENDFORCE(-2, 0, 0);
  BENDFORCE(0, 2, 0);
	BENDFORCE(0, -2, 0);
	BENDFORCE(0, 0, 2);
	BENDFORCE(0, 0, -2);

  //add structural, shear and bend force 
	pSUM(*F, Fstruct, *F);
	pSUM(*F, Fshear, *F);
	pSUM(*F, Fbend, *F);			
}

//Compute the external force at arbitrary position, using trilinear interpolation
void computeExternalForce(struct world* jello, int i, int j, int k, point* F)
{
  point c000, c001, c010, c011, c100, c101, c110, c111;
  int resolution = jello->resolution;

  //xd = (x-x0)/(x1-x0);
  // yd = (y-y0)/(y1-y0);
  // zd = (z-z0)/(z1-z0);

  double xd, yd, zd;
  int i0, j0, k0, i1, j1, k1;
  point f1,f2,f3;

  // map the points to the force field 
  i0=(jello->p[i][j][k].x+2)*jello->resolution/4;
  j0=(jello->p[i][j][k].y+2)*jello->resolution/4;
  k0=(jello->p[i][j][k].z+2)*jello->resolution/4;

  i1=((jello->p[i][j][k].x+2)*jello->resolution/4)+1;
  j1=((jello->p[i][j][k].y+2)*jello->resolution/4)+1;
  k1=((jello->p[i][j][k].z+2)*jello->resolution/4)+1;

  xd=((jello->p[i][j][k].x+2.0)-4.0/jello->resolution*i0)/(4.0/jello->resolution);
  yd=((jello->p[i][j][k].y+2.0)-4.0/jello->resolution*j0)/(4.0/jello->resolution);
  zd=((jello->p[i][j][k].z+2.0)-4.0/jello->resolution*k0)/(4.0/jello->resolution);

  // get 8 surrounding points 
  c000 = jello->forceField[i0 * resolution * resolution + j0 * resolution + k0];
  c001 = jello->forceField[i0 * resolution * resolution + j0 * resolution + k1];
  c010 = jello->forceField[i0 * resolution * resolution + j1 * resolution + k0];
  c011 = jello->forceField[i0 * resolution * resolution + j1 * resolution + k1];
  c100 = jello->forceField[i1 * resolution * resolution + j0 * resolution + k0];
  c101 = jello->forceField[i1 * resolution * resolution + j0 * resolution + k1];
  c110 = jello->forceField[i1 * resolution * resolution + j1 * resolution + k0]; 
  c111 = jello->forceField[i1 * resolution * resolution + j1 * resolution + k1];

  pDIFFERENCE(c100,c000,f1);pMULTIPLY(f1,xd,f1);pSUM(c000,f1,f1);
  pDIFFERENCE(c101,c001,f2);pMULTIPLY(f2,xd,f2);pSUM(c001,f2,f2);
  pDIFFERENCE(f2,f1,f3);pMULTIPLY(f3,zd,f3);pSUM(f1,f3,f3);
  pDIFFERENCE(c110,c010,f1);pMULTIPLY(f1,xd,f1);pSUM(c010,f1,f1);
  pDIFFERENCE(c111,c011,f2);pMULTIPLY(f2,xd,f2);pSUM(c011,f2,f2);
  pDIFFERENCE(f2,f1,f2);pMULTIPLY(f2,zd,f2);pSUM(f1,f2,f2);
  pDIFFERENCE(f2,f3,f2);pMULTIPLY(f2,yd,f2);pSUM(f2,f3,f1);
  pSUM(*F,f1,*F);

}

//If there is an inclinded plane, compute the plane collision force
void computePlaneCollision(struct world* jello, int i, int j, int k, point* F)
{
  double a, b, c, d, distance, R= 0;
  double ks = jello->kCollision, kd = jello->dCollision;
  point normal, f;
  point L= {0, 0, 0}, V={0, 0, 0};
  bool isCollide = false;
  memset(&f, 0, sizeof(point));
 // memset(F, 0, sizeof(point));

  V = jello->v[i][j][k];

  a = jello->a;
  b = jello->b;
  c = jello->c;
  d = jello->d;

  normal.x = a;
  normal.y = b;
  normal.z = c;

  if(jello->incPlanePresent)
  {
    distance = (a*jello->p[i][j][k].x + b*jello->p[i][j][k].y + c*jello->p[i][j][k].z +d)/
    sqrt(pow(a,2)+pow(b,2)+pow(c,2));
    if(distance <= 0) isCollide = true;
    if(isCollide)
    {
      double length;
      pNORMALIZE(normal);
      pMULTIPLY(normal, distance, L);
      springForce(L, V, &f, 1.0 / 7*R, ks, kd); 
      pSUM(*F, f, *F);
    }
  }
}

double magnitude(point p)
{
	double res;
	res = sqrt(pow(p.x,2)+pow(p.y,2)+pow(p.z,2));
	return res;
}

// Compute mouse drag forces(drag on certain simulation point)
void computeMouseDragForce(struct world* jello, int i, int j, int k, point* F)
{
    //Need to get the mouse click cube point position and final released position
    // dragging vector
  point drag;
  point f;
  memset(&f, 0, sizeof(point));
  drag.x = i;
  drag.y = j;
  drag.z = k;
	pDIFFERENCE(drag,z_hitPos,drag);
	if (z_hit && magnitude(drag) <= 3)
  {
	  pMULTIPLY(z_dragForce, 1-magnitude(drag)/4, f);
	  pSUM(*F, f, *F);
  }	
}
// Computer box collision for six faces
void computeBoxCollision(struct world* jello, int i, int j, int k, point* F)
{
  double ks = jello->kCollision;
  double kd = jello -> dCollision;
  double R = 0.0;
  point f = {0,0,0};
 
  if(jello->p[i][j][k].x > 2.0) 
  {
		//Hook force & damping force
		point L, V, B;
		B.x = 2.0;
    B.y = jello->p[i][j][k].y;
    B.z = jello->p[i][j][k].z;

	  pDIFFERENCE(jello->p[i][j][k], B, L);
		V = jello->v[i][j][k];
    springForce(L, V, &f, 1.0 / 7*R, ks, kd); 
    pSUM(*F, f, *F);
	}

  if(jello->p[i][j][k].x < -2.0) 
  {
		//Hook force & damping force
		point L, V;
		point B;
    B.x = -2.0;
    B.y = jello->p[i][j][k].y;
    B.z = jello->p[i][j][k].z;

	  pDIFFERENCE(jello->p[i][j][k], B, L);
		V = jello->v[i][j][k];
    springForce(L, V, &f, 1.0 / 7*R, ks, kd); 
    pSUM(*F, f, *F);
	}

  if(jello->p[i][j][k].y > 2.0) 
  {
		//Hook force & damping force
		point L, V;
		point B;
    B.x = jello->p[i][j][k].x;
    B.y = 2.0;
    B.z = jello->p[i][j][k].z;

	  pDIFFERENCE(jello->p[i][j][k], B, L);
		V = jello->v[i][j][k];
    springForce(L, V, &f, 1.0 / 7*R, ks, kd); 
    pSUM(*F, f, *F);
	}

  if(jello->p[i][j][k].y < -2.0) 
  {
		//Hook force & damping force
		point L, V;
		point B;
    B.x = jello->p[i][j][k].x;
    B.y = -2.0;
    B.z = jello->p[i][j][k].z;

	  pDIFFERENCE(jello->p[i][j][k], B, L);
		V = jello->v[i][j][k];
    springForce(L, V, &f, 1.0 / 7*R, ks, kd); 
    pSUM(*F, f, *F); 
	}

  if(jello->p[i][j][k].z > 2.0) 
  {
		//Hook force & damping force
		point L, V;
		point B;
    B.x = jello->p[i][j][k].x;
    B.y = jello->p[i][j][k].y;
    B.z = 2.0;

	  pDIFFERENCE(jello->p[i][j][k], B, L);
		V = jello->v[i][j][k];
    springForce(L, V, &f, 1.0 / 7*R, ks, kd); 
    pSUM(*F, f, *F);
	}

  if(jello->p[i][j][k].z < -2.0) 
  {
		//Hook force & damping force
		point L, V;
		point B;
    B.x = jello->p[i][j][k].x;
    B.y = jello->p[i][j][k].y;
    B.z = -2.0;

	  pDIFFERENCE(jello->p[i][j][k], B, L);
		V = jello->v[i][j][k];
    springForce(L, V, &f, 1.0 / 7*R, ks, kd); 
    pSUM(*F, f, *F);
	}
}

void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  //compute acceration for structral, shear and bend springs
  //for structral spring F = m * a

  for(int i = 0; i<8; i++)
  {
    for(int j = 0; j<8; j++)
    {
      for(int k = 0; k<8; k++)
      {
        point F={0,0,0};
        memset(&F, 0, sizeof(point));  
        computeSpringForce(jello,i,j,k,&F);
        computeBoxCollision(jello,i,j,k,&F);
        computePlaneCollision(jello,i,j,k,&F);
        computeExternalForce(jello, i, j, k, &F);
        computeMouseDragForce(jello, i, j, k, &F);
        pMULTIPLY(F, 1.0 / jello->mass, a[i][j][k]);
      }
    }
  }
}			
			
/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;
      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
