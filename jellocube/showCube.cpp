/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "showCube.h"
#include<iostream>
using namespace std;

int pointMap(int side, int i, int j)
{
  int r;

  switch (side)
  {
  case 1: //[i][j][0] bottom face
    r = 64 * i + 8 * j;
    break;
  case 6: //[i][j][7] top face
    r = 64 * i + 8 * j + 7;
    break;
  case 2: //[i][0][j] front face
    r = 64 * i + j;
    break;
  case 5: //[i][7][j] back face
    r = 64 * i + 56 + j;
    break;
  case 3: //[0][i][j] left face
    r = 8 * i + j;
    break;
  case 4: //[7][i][j] right face
    r = 448 + 8 * i + j;
    break;
  }

  return r;
}

void showCube(struct world * jello)
{
  int i,j,k,ip,jp,kp;
  point r1,r2,r3; // aux variables
  
  /* normals buffer and counter for Gourad shading*/
  struct point normal[8][8];
  int counter[8][8];

  int face;
  double faceFactor, length;

  if (fabs(jello->p[0][0][0].x) > 10)
  {
    printf ("Your cube somehow escaped way out of the box.\n");
    exit(0);
  }


  
  #define NODE(face,i,j) (*((struct point * )(jello->p) + pointMap((face),(i),(j))))

  
  #define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if\
    (!( (ip>7) || (ip<0) ||\
      (jp>7) || (jp<0) ||\
    (kp>7) || (kp<0) ) && ((i==0) || (i==7) || (j==0) || (j==7) || (k==0) || (k==7))\
       && ((ip==0) || (ip==7) || (jp==0) || (jp==7) || (kp==0) || (kp==7))) \
    {\
      glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);\
      glVertex3f(jello->p[ip][jp][kp].x,jello->p[ip][jp][kp].y,jello->p[ip][jp][kp].z);\
    }\

 if (viewingMode==0 && isTexture==0) // render wireframe
	{
		glLineWidth(1);
		glPointSize(5);
		glDisable(GL_LIGHTING);
		for (i=0; i<=7; i++)
			for (j=0; j<=7; j++)
				for (k=0; k<=7; k++)
				{
					if (i*j*k*(7-i)*(7-j)*(7-k) != 0) // not surface point
						continue;
					if (mode == GL_SELECT)
			  	glLoadName(i * 8 * 8 + j * 8 + k);
					glBegin(GL_POINTS); // draw point
					glColor4f(0,0,0,0);  
					glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);        
					glEnd();
				
					glBegin(GL_LINES);      
					// structural
					if (structural == 1)
					{
						glColor4f(0,0,1,1);
						PROCESS_NEIGHBOUR(1,0,0);
						PROCESS_NEIGHBOUR(0,1,0);
						PROCESS_NEIGHBOUR(0,0,1);
						PROCESS_NEIGHBOUR(-1,0,0);
						PROCESS_NEIGHBOUR(0,-1,0);
						PROCESS_NEIGHBOUR(0,0,-1);
					}

					// shear
					if (shear == 1)
					{
						glColor4f(0,1,0,1);
						PROCESS_NEIGHBOUR(1,1,0);
						PROCESS_NEIGHBOUR(-1,1,0);
						PROCESS_NEIGHBOUR(-1,-1,0);
						PROCESS_NEIGHBOUR(1,-1,0);
						PROCESS_NEIGHBOUR(0,1,1);
						PROCESS_NEIGHBOUR(0,-1,1);
						PROCESS_NEIGHBOUR(0,-1,-1);
						PROCESS_NEIGHBOUR(0,1,-1);
						PROCESS_NEIGHBOUR(1,0,1);
						PROCESS_NEIGHBOUR(-1,0,1);
						PROCESS_NEIGHBOUR(-1,0,-1);
						PROCESS_NEIGHBOUR(1,0,-1);

						PROCESS_NEIGHBOUR(1,1,1)
						PROCESS_NEIGHBOUR(-1,1,1)
						PROCESS_NEIGHBOUR(-1,-1,1)
						PROCESS_NEIGHBOUR(1,-1,1)
						PROCESS_NEIGHBOUR(1,1,-1)
						PROCESS_NEIGHBOUR(-1,1,-1)
						PROCESS_NEIGHBOUR(-1,-1,-1)
						PROCESS_NEIGHBOUR(1,-1,-1)
					}

					// bend
					if (bend == 1)
					{
						glColor4f(1,0,0,1);
						PROCESS_NEIGHBOUR(2,0,0);
						PROCESS_NEIGHBOUR(0,2,0);
						PROCESS_NEIGHBOUR(0,0,2);
						PROCESS_NEIGHBOUR(-2,0,0);
						PROCESS_NEIGHBOUR(0,-2,0);
						PROCESS_NEIGHBOUR(0,0,-2);
					}
					glEnd();
				}
		glEnable(GL_LIGHTING);
	}
	else 
	{ // viewingMode == 1
		// if texture is on
		if(!isTexture) 
		{
		glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBindTexture(GL_TEXTURE_2D,jello->texture[0]);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

			for (face=1; face <= 6; face++) 
			// face == face of a cube
			// 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top
			{
				if ((face==1) || (face==3) || (face==5))
					faceFactor=-1; // flip orientation
				else
					faceFactor=1;
				
				for (i=0; i <= 7; i++) // reset buffers
					for (j=0; j <= 7; j++)
				{
					normal[i][j].x=0;normal[i][j].y=0;normal[i][j].z=0;
					counter[i][j]=0;
				}

				/* process triangles, accumulate normals for Gourad shading */

				for (i=0; i <= 6; i++)
					for (j=0; j <= 6; j++) // process block (i,j)
					{
						pDIFFERENCE(NODE(face,i+1,j),NODE(face,i,j),r1); // first triangle
						pDIFFERENCE(NODE(face,i,j+1),NODE(face,i,j),r2);
						CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
						pNORMALIZE(r3);
						pSUM(normal[i+1][j],r3,normal[i+1][j]);
						counter[i+1][j]++;
						pSUM(normal[i][j+1],r3,normal[i][j+1]);
						counter[i][j+1]++;
						pSUM(normal[i][j],r3,normal[i][j]);
						counter[i][j]++;

						pDIFFERENCE(NODE(face,i,j+1),NODE(face,i+1,j+1),r1); // second triangle
						pDIFFERENCE(NODE(face,i+1,j),NODE(face,i+1,j+1),r2);
						CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
						pNORMALIZE(r3);
						pSUM(normal[i+1][j],r3,normal[i+1][j]);
						counter[i+1][j]++;
						pSUM(normal[i][j+1],r3,normal[i][j+1]);
						counter[i][j+1]++;
						pSUM(normal[i+1][j+1],r3,normal[i+1][j+1]);
						counter[i+1][j+1]++;
					}

				/* the actual rendering */
				for (j=1; j<=7; j++) 
				{

					if (faceFactor  > 0)
						glFrontFace(GL_CCW); // the usual definition of front face
					else
						glFrontFace(GL_CW); // flip definition of orientation

					glBegin(GL_TRIANGLE_STRIP);
					for (i=0; i<=7; i++)
					{
						glTexCoord2f(1.0 * i / 7, 1.0 * j / 7);
						glNormal3f(normal[i][j].x / counter[i][j],normal[i][j].y / counter[i][j],
							normal[i][j].z / counter[i][j]);
						glVertex3f(NODE(face,i,j).x, NODE(face,i,j).y, NODE(face,i,j).z);
					
						glTexCoord2f(1.0 * i / 7, (1.0 * j-1) / 7);
						glNormal3f(normal[i][j-1].x / counter[i][j-1],normal[i][j-1].y/ counter[i][j-1],
							normal[i][j-1].z / counter[i][j-1]);
						glVertex3f(NODE(face,i,j-1).x, NODE(face,i,j-1).y, NODE(face,i,j-1).z);
					}
					glEnd();
				}
			} // end for loop over faces
		glDisable(GL_TEXTURE_2D);
		//	glPopAttrib();
		}
		else { // texture off
			glPolygonMode(GL_FRONT, GL_FILL); 

			for (face=1; face <= 6; face++) 
			// face == face of a cube
			// 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top
			{
				if ((face==1) || (face==3) || (face==5))
					faceFactor=-1; // flip orientation
				else
					faceFactor=1;


				for (i=0; i <= 7; i++) // reset buffers
					for (j=0; j <= 7; j++)
				{
					normal[i][j].x=0;normal[i][j].y=0;normal[i][j].z=0;
					counter[i][j]=0;
				}

				/* process triangles, accumulate normals for Gourad shading */

				for (i=0; i <= 6; i++)
					for (j=0; j <= 6; j++) // process block (i,j)
					{
						pDIFFERENCE(NODE(face,i+1,j),NODE(face,i,j),r1); // first triangle
						pDIFFERENCE(NODE(face,i,j+1),NODE(face,i,j),r2);
						CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
						pNORMALIZE(r3);
						pSUM(normal[i+1][j],r3,normal[i+1][j]);
						counter[i+1][j]++;
						pSUM(normal[i][j+1],r3,normal[i][j+1]);
						counter[i][j+1]++;
						pSUM(normal[i][j],r3,normal[i][j]);
						counter[i][j]++;

						pDIFFERENCE(NODE(face,i,j+1),NODE(face,i+1,j+1),r1); // second triangle
						pDIFFERENCE(NODE(face,i+1,j),NODE(face,i+1,j+1),r2);
						CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
						pNORMALIZE(r3);
						pSUM(normal[i+1][j],r3,normal[i+1][j]);
						counter[i+1][j]++;
						pSUM(normal[i][j+1],r3,normal[i][j+1]);
						counter[i][j+1]++;
						pSUM(normal[i+1][j+1],r3,normal[i+1][j+1]);
						counter[i+1][j+1]++;
					}

				/* the actual rendering */
				for (j=1; j<=7; j++) 
				{

					if (faceFactor  > 0)
						glFrontFace(GL_CCW); // the usual definition of front face
					else
						glFrontFace(GL_CW); // flip definition of orientation

					glBegin(GL_TRIANGLE_STRIP);
					for (i=0; i<=7; i++)
					{
						glNormal3f(normal[i][j].x / counter[i][j],normal[i][j].y / counter[i][j],
							normal[i][j].z / counter[i][j]);
						glVertex3f(NODE(face,i,j).x, NODE(face,i,j).y, NODE(face,i,j).z);

						glNormal3f(normal[i][j-1].x / counter[i][j-1],normal[i][j-1].y/ counter[i][j-1],
							normal[i][j-1].z / counter[i][j-1]);
						glVertex3f(NODE(face,i,j-1).x, NODE(face,i,j-1).y, NODE(face,i,j-1).z);
					}
					glEnd();
				}
			} // end for loop over faces
		} // end-else
	}
	glFrontFace(GL_CCW);
}

void showBoundingBox()
{
  int i,j;

  glColor4f(0.6,0.6,0.6,0);

  glBegin(GL_LINES);

  // front face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(i,-2,-2);
    glVertex3f(i,-2,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,-2,j);
    glVertex3f(2,-2,j);
  }

  // back face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(i,2,-2);
    glVertex3f(i,2,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,2,j);
    glVertex3f(2,2,j);
  }

  // left face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(-2,i,-2);
    glVertex3f(-2,i,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,-2,j);
    glVertex3f(-2,2,j);
  }

  // right face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(2,i,-2);
    glVertex3f(2,i,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(2,-2,j);
    glVertex3f(2,2,j);
  }
  
  glEnd();
  return;
}

point assignValue(double x, double y, double z)
{
  point res;
  res.x = x;
  res.y = y; 
  res.z = z;
  return res;
}

// Rendering inclined plane
void showInclinedPlane(struct world * jello)
{
 // printf("show inclined plane!");
  if(!jello->incPlanePresent) return;

  double a=jello->a,b=jello->b,c=jello->c,d=jello->d;
  double length;
  double x, y, z;
  
  // a*x + b*y + c*z + d = 0
  point plane[12];
    
  z =(-d-a*2-b*2)/c;
  plane[0]= assignValue(2, 2, z);
  z =(-d-a*2-b*(-2))/c;
  plane[1] = assignValue(2, -2, z);
  z =(-d-a*(-2)-b*2)/c;
  plane[2] = assignValue(-2, 2, z);
  z =(-d-a*(-2)-b*(-2))/c;
  plane[3] = assignValue(-2, -2, z);
    
  y =(-d-a*2-c*2)/b;
  plane[4] = assignValue(2, y, 2);
  y =(-d-a*2-c*(-2))/b;
  plane[5] = assignValue(2, y, -2);
  y =(-d-a*(-2)-c*2)/b;
  plane[6] = assignValue(-2, y, 2);
  y =(-d-a*(-2)-c*(-2))/b;
  plane[7] = assignValue(-2, y, -2);
    
  x =(-d-b*2-c*2)/a;
  plane[8] = assignValue(x, 2, 2);
  x =(-d-b*2-c*(-2))/a;
  plane[9] = assignValue(x, 2, -2);
  x =(-d-b*(-2)-c*2)/a;
  plane[10] = assignValue(x, -2, 2);
  x =(-d-b*(-2)-c*(-2))/a;
  plane[11] = assignValue(x, -2, -2);

  glEnable(GL_LIGHTING);  
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
          
  glBegin(GL_TRIANGLE_STRIP);
  glColor4f(0.3, 0.3, 0.3, 0);
            
  for(int i =0;i<12;i++)
  {
    if(plane[i].x >=-2 && plane[i].x <=2 && plane[i].y >=-2 && plane[i].y <=2 && plane[i].z >=-2 && plane[i].z <=2)
    glVertex3f(plane[i].x, plane[i].y, plane[i].z);
  }

  glEnd();     
  glEnable(GL_CULL_FACE);
  glDisable(GL_LIGHTING);
}
