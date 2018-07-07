/*
  Ass3.c
  Fangye Tang
  B00612172

  Press R to swich camera and Q to quit
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <GL/glut.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// The number of segments per curve.
#define NUMBER_SEGMENTS 100

// The maximum number of data points the roller coaster can have.
#define MAX_POINTS 100

// Display Callbacks
static void myTimer(int value);
static void	myDisplay(void);
static void	myReshape(int w, int h);
static void keyPress(int key, int x, int y);
static void keyRelease(int key, int x, int y);
static void myKey(unsigned char key, int x, int y);

// Initialize functions for roller coaster.
static void	init(void);

static double ctrlpoints[MAX_POINTS][3];
static double xMax, yMax;
static double phi = 0;
static double view_phi = 0;//use to control left and right's view
static double** q;//B-spline curve
static double** dq;//store first derivative
static double** ddq;//store second derivative
static double** u;//store u vector for each point
static double** v;//store v vector
static double** n;//store n vector
static double** up;//store up vector
static double* k;//store k vector which is the y-component of r*s
// The number of control points in this data set.
static int numberPoints = 0;
// Used to choose camera mode.
static int cameraMode = 1;
// The current position of the roller coaster camera.
static int currentPosition = 0;
// The current velocity of the roller coaster camera.
static int velocity = 0;

//Display list that will hold th epoints and the drawing.
static GLuint scene;

// Helper functions
static void calculateVectors(void);//main function for calculating something
static int loadSplines();//load control points from "Control_Point.txt"
static double *crossProduct(double *A, double *B);//implement cross product between two vectors
static double *unit(double *A);//cacluate a unit vector
static double *negative(double *A);//implement u=-u
static double *BSpline(double u, double *P0, double *P1, double *P2, double *P3);// cacluate B-Spline's point
static double *dBSpline(double u, double *P0, double *P1, double *P2, double *P3);//cacluate the first derivative
static double *ddBSpline(double u, double *P0, double *P1, double *P2, double *P3);//cacluate the second derivative
static double *MatrixMultiplication(double v[3], double M[3][3]);//implement matrix multiplication

static void drawCurve();//draw three rails
static void drawConnectors();//draw connectors between two rails
static void drawColumns();//draw columns to ground
static void drawGround();//draw ground plan which is height -40
static void drawSkybox();
static void initLighting();

/* Run through the main section of code for the program, this will simulate a roller coaster and change
cameras based on user input.*/
int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(1000, 600);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Roller Coaster");

	glutSpecialFunc(keyPress);
	glutSpecialUpFunc(keyRelease);
	glutKeyboardFunc(myKey);

	// Set up the data points to use with the coaster and the display lists.
	init();

	glutDisplayFunc(myDisplay);
	glutReshapeFunc(myReshape);
	glutTimerFunc(33, myTimer, 0);

	glClearColor(0.0, 0.0, 0.55, 1.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glutMainLoop();

	return 0;
}

/* Initialize anything necessary to set up the scene for the roller coaster simulation. */
void init(void){
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);	glEnable(GL_COLOR_MATERIAL);	glEnable(GL_NORMALIZE);	/*if don't want light, just delete following 3 lines*/	glEnable(GL_LIGHTING);	glEnable(GL_LIGHT0);	glEnable(GL_LIGHT1);
	initLighting();
	loadSplines();
	// Read in the control points from a file, first lets test without that feature.
	q = (double **)malloc(MAX_POINTS*NUMBER_SEGMENTS * sizeof(double *));
	dq = (double **)malloc(MAX_POINTS*NUMBER_SEGMENTS * sizeof(double *));
	ddq = (double **)malloc(MAX_POINTS*NUMBER_SEGMENTS * sizeof(double *));
	u = (double **)malloc(MAX_POINTS*NUMBER_SEGMENTS * sizeof(double *));
	v = (double **)malloc(MAX_POINTS*NUMBER_SEGMENTS * sizeof(double *));
	n = (double **)malloc(MAX_POINTS*NUMBER_SEGMENTS * sizeof(double *));
	up = (double **)malloc(MAX_POINTS*NUMBER_SEGMENTS * sizeof(double *));
	k = (double *)malloc(MAX_POINTS*NUMBER_SEGMENTS * sizeof(double));
	calculateVectors();

	// Generate a display list that will hold the scene.
	scene = glGenLists(1);
	glNewList(scene, GL_COMPILE);	
	// Draw the ground and colour it green.
	drawGround();

	// Draw the coaster.
	drawCurve();

	//Draw connector which attach the rails to each other.
	drawConnectors();

	//Draw columns which attach to the ground.
	drawColumns();

	//Draw sky
	drawSkybox();

	glEndList();
}

int loadSplines() {
	/* load the control point file */
	FILE *file = fopen("Control_Point.txt", "r");
	if (file == NULL){
		printf("file does not exists.");
		return 0;
	}
	/*read control points to data*/
	fseek(file, 0, SEEK_END);
	int len = ftell(file);
	char data[1000];
	for (int i = 0; i < 1000; i++)
		data[i] = 0;
	rewind(file);
	fread(data, 1, len, file);
	data[len] = 0;
	//store the number of control points
	int i = 0;
	char *token;
	token = strtok(data, "\n");
	numberPoints = atoi(token);
	//store each point to ctrlpoints array
	while (token != NULL){
		token = strtok(NULL, ",");
		if (token == NULL)
			break;
		ctrlpoints[i][0] = atof(token);
		token = strtok(NULL, ",");
		ctrlpoints[i][1] = atof(token);
		token = strtok(NULL, "\n");
		ctrlpoints[i][2] = atof(token);
		i++;
	}
	free(token);
	if (i != numberPoints){
		printf("error of adding control points");
	}
	/*make last three control points repeat, so that it will be a continuous curve*/
	ctrlpoints[numberPoints][0] = ctrlpoints[0][0];
	ctrlpoints[numberPoints][1] = ctrlpoints[0][1];
	ctrlpoints[numberPoints][2] = ctrlpoints[0][2];

	ctrlpoints[numberPoints+1][0] = ctrlpoints[1][0];
	ctrlpoints[numberPoints+1][1] = ctrlpoints[1][1];
	ctrlpoints[numberPoints+1][2] = ctrlpoints[1][2];

	ctrlpoints[numberPoints+2][0] = ctrlpoints[2][0];
	ctrlpoints[numberPoints+2][1] = ctrlpoints[2][1];
	ctrlpoints[numberPoints+2][2] = ctrlpoints[2][2];

	numberPoints += 3;
	fclose(file);
	return 0;
}

// Calculate the vertices of the curve.
void calculateVectors(){
	int index = 0;

	for (int j = 0; j < numberPoints - 3; j++){
		for (int i = 0; i <= NUMBER_SEGMENTS; i++){
			q[(j*NUMBER_SEGMENTS + i)] = BSpline((double)i / NUMBER_SEGMENTS, ctrlpoints[j], ctrlpoints[j + 1], ctrlpoints[j + 2], ctrlpoints[j + 3]);
			dq[(j*NUMBER_SEGMENTS + i)] = dBSpline((double)i / NUMBER_SEGMENTS, ctrlpoints[j], ctrlpoints[j + 1], ctrlpoints[j + 2], ctrlpoints[j + 3]);
			ddq[(j*NUMBER_SEGMENTS + i)] = ddBSpline((double)i / NUMBER_SEGMENTS, ctrlpoints[j], ctrlpoints[j + 1], ctrlpoints[j + 2], ctrlpoints[j + 3]);

			/*
			  r={dq[0],-40,dq[2]}
			  s={ddq[0],-40,ddq[2]}
			*/
			double divisor = sqrt(dq[(j*NUMBER_SEGMENTS + i)][0] * dq[(j*NUMBER_SEGMENTS + i)][0] + -40*-40 + dq[(j*NUMBER_SEGMENTS + i)][2] * dq[(j*NUMBER_SEGMENTS + i)][2]);
			k[(j*NUMBER_SEGMENTS + i)] = (dq[(j*NUMBER_SEGMENTS + i)][2] * ddq[(j*NUMBER_SEGMENTS + i)][0] - dq[(j*NUMBER_SEGMENTS + i)][0] * ddq[(j*NUMBER_SEGMENTS + i)][2]) / (divisor*divisor*divisor);

			n[(j*NUMBER_SEGMENTS + i)] = unit(negative(dq[(j*NUMBER_SEGMENTS + i)]));
			double UP[3] = { 0, 1, 0 };
			double c = cos(100*k[(j*NUMBER_SEGMENTS + i)]);
			double s = sin(100*k[(j*NUMBER_SEGMENTS + i)]);
			double n1 = n[(j*NUMBER_SEGMENTS + i)][0];
			double n2 = n[(j*NUMBER_SEGMENTS + i)][1];
			double n3 = n[(j*NUMBER_SEGMENTS + i)][2];
			double rotate[3][3] = { { (1 - c)*n1*n1 + c, (1 - c)*n1*n2 - s*n3, (1 - c)*n1*n3 + s*n2 },
									{ (1 - c)*n1*n2 + s*n3, (1 - c)*n2*n2 + c, (1 - c)*n1*n3 - s*n1 },
									{ (1 - c)*n1*n3 - s*n2, (1 - c)*n2*n3 + s*n1, (1 - c)*n3*n3 + c } };
			up[(j*NUMBER_SEGMENTS + i)] = MatrixMultiplication( UP, rotate );

			u[(j*NUMBER_SEGMENTS + i)] = unit(crossProduct(up[(j*NUMBER_SEGMENTS + i)], n[(j*NUMBER_SEGMENTS + i)]));
			v[(j*NUMBER_SEGMENTS + i)] = unit(crossProduct(n[(j*NUMBER_SEGMENTS + i)], u[(j*NUMBER_SEGMENTS + i)]));

		}
	}

}

/*get every q(u) using B-spline curve*/
double *BSpline(double u, double *P0, double *P1, double *P2, double *P3){
	double *C = malloc(3 * sizeof(double));
	if (C == NULL){
		fprintf(stderr, "out of memory\n");
		exit(EXIT_FAILURE);
	}

	double r0 = (1.0 / 6.0)*u*u*u;
	double r1 = (1.0 / 6.0)*(-3 * u*u*u + 3 * u*u + 3 * u + 1);
	double r2 = (1.0 / 6.0)*(3 * u*u*u - 6 * u*u + 4);
	double r3 = (1.0 / 6.0)*(1 - u)*(1 - u)*(1 - u);

	C[0] = P0[0] * r3 + P1[0] * r2 + P2[0] * r1 + P3[0] * r0;
	C[1] = P0[1] * r3 + P1[1] * r2 + P2[1] * r1 + P3[1] * r0;
	C[2] = P0[2] * r3 + P1[2] * r2 + P2[2] * r1 + P3[2] * r0;

	return C;
}
/*find first derivative*/
double *dBSpline(double u, double *P0, double *P1, double *P2, double *P3){
	double *C = malloc(3 * sizeof(double));
	if (C == NULL){
		fprintf(stderr, "out of memory\n");
		exit(EXIT_FAILURE);
	}

	double r0 = (1.0 / 2.0)*u*u;
	double r1 = ((-3.0 / 2.0)*u*u) + u + ((1.0 / 2.0));
	double r2 = ((3.0 / 2.0)*u*u) - 2 * u;
	double r3 = ((-1.0 / 2.0)*u*u) + u + ((-1.0 / 2.0));

	C[0] = P0[0] * r3 + P1[0] * r2 + P2[0] * r1 + P3[0] * r0;
	C[1] = P0[1] * r3 + P1[1] * r2 + P2[1] * r1 + P3[1] * r0;
	C[2] = P0[2] * r3 + P1[2] * r2 + P2[2] * r1 + P3[2] * r0;

	return C;
}
/*find second derivative*/
double *ddBSpline(double u, double *P0, double *P1, double *P2, double *P3){
	double *C = malloc(3 * sizeof(double));
	if (C == NULL){
		fprintf(stderr, "out of memory\n");
		exit(EXIT_FAILURE);
	}

	double r0 = u;
	double r1 = (-3.0*u) + 1;
	double r2 = ((3.0)*u) - 2;
	double r3 = (-u) + 1;

	C[0] = P0[0] * r3 + P1[0] * r2 + P2[0] * r1 + P3[0] * r0;
	C[1] = P0[1] * r3 + P1[1] * r2 + P2[1] * r1 + P3[1] * r0;
	C[2] = P0[2] * r3 + P1[2] * r2 + P2[2] * r1 + P3[2] * r0;


	return C;
}
/*implement u=-u*/
double *negative(double *A){
	double *B = malloc(3 * sizeof(double));
	if (B == NULL){
		fprintf(stderr, "out of memory\n");
		exit(EXIT_FAILURE);
	}

	B[0] = -1.0*A[0];
	B[1] = -1.0*A[1];
	B[2] = -1.0*A[2];
	return B;
}
/* implement u / ||u|| */
double *unit(double *A){
	double *B = malloc(3 * sizeof(double));
	if (B == NULL){
		fprintf(stderr, "out of memory\n");
		exit(EXIT_FAILURE);
	}

	double divisor = sqrt(A[0] * A[0] + A[1] * A[1] + A[2] * A[2]);
	if (divisor > 0){
		B[0] = A[0] / divisor;
		B[1] = A[1] / divisor;
		B[2] = A[2] / divisor;
	}
	else{
		B[0] = A[0];
		B[1] = A[1];
		B[2] = A[2];
	}
	return B;
}

/*implement cross product*/
double *crossProduct(double *A, double *B){
	double *C = malloc(3 * sizeof(double));
	if (C == NULL){
		fprintf(stderr, "out of memory\n");
		exit(EXIT_FAILURE);
	}

	C[0] = A[1] * B[2] - A[2] * B[1];
	C[1] = A[2] * B[0] - A[0] * B[2];
	C[2] = A[0] * B[1] - A[1] * B[0];
	return C;
}
/*implement matrix multiplication (only for 4*4 times 4*1)*/
/*
   M[0][0] M[0][1] M[0][2]        v[0]
   M[1][0] M[1][1] M[1][2]  *     v[1]
   M[2][0] M[2][1] M[2][2]        v[2]
*/
double *MatrixMultiplication(double v[3], double M[3][3]){
	double *C = malloc(3 * sizeof(double));
	if (C == NULL){
		fprintf(stderr, "out of memory\n");
		exit(EXIT_FAILURE);
	}
	C[0] = M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2];
	C[1] = M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2];
	C[2] = M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2];
	return C;
}

/* Draw large planes as ground*/
void drawGround(){
	glBegin(GL_POLYGON);
	glColor3f(0.1, 0.83, 0.1);
	glVertex3f(200, -40, 200);
	glVertex3f(200, -40, -200);
	glVertex3f(-200, -40, -200);
	glVertex3f(-200, -40, 200);
	glEnd();
}

/*Set lighting*/
void initLighting(){
	GLfloat ambient[] = { 0.4, 0.2, 0.2, 1.0 };
	GLfloat diffuse[] = { 0.8, 0.4, 0.4, 1.0 };
	GLfloat specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat position[] = { 0.0, 2.0, 1.0, 1.0 };
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	GLfloat position1[] = { 1.0, 2.0, 0.0, 1.0 };
	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT1, GL_POSITION, position1);
}

/* Draw uniform B-spline curves for roller coaster's three rails.*/
void drawCurve()
{
	//Light stuff
	GLfloat specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat shine[] = { 50.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, shine);
	//Left rail which start point in (q[j][0]-u[j][0],q[j][1]-0.1*v[j][q],q[j][2]+u[j][2])
	glBegin(GL_QUAD_STRIP);
	for (int j = 0; j < (numberPoints*NUMBER_SEGMENTS) - 300; j++){
		for (int i = 0; i <= 10; i++){
			glColor3f(0.96, 0.0, 0.96);
			glVertex3f(q[j][0] - u[j][0] + u[j][0] * cos(2 * M_PI / 10 * i)*0.2, q[j][1] - 0.1*v[j][1] + v[j][1] * sin(2 * M_PI / 10 * i)*0.2, q[j][2] - u[j][2] + u[j][2] * cos(2 * M_PI / 10 * i)*0.2);
			glVertex3f(q[j + 1][0] - u[j + 1][0] + u[j + 1][0] * cos(2 * M_PI / 10 * i)*0.2, q[j + 1][1] - 0.1*v[j + 1][1] + v[j + 1][1] * sin(2 * M_PI / 10 * i)*0.2, q[j + 1][2] - u[j+1][2] + u[j+1][2] * cos(2 * M_PI / 10 * i)*0.2);
		}
	}
	glEnd();
	//Right Rail which start point in (q[j][0]+u[j][0],q[j][1]-0.1*v[j][q],q[j][2]+u[j][2])
	glBegin(GL_QUAD_STRIP);
	for (int j = 0; j < (numberPoints*NUMBER_SEGMENTS) - 300; j++){
		for (int i = 0; i <= 10; i++){
			glColor3f(0.96, 0.0, 0.96);
			glVertex3f(q[j][0] + u[j][0] + u[j][0] * cos(2 * M_PI / 10 * i)*0.2, q[j][1] - 0.1*v[j][1] + v[j][1] * sin(2 * M_PI / 10 * i)*0.2, q[j][2] + u[j][2] + u[j][2] * cos(2 * M_PI / 10 * i)*0.2);
			glVertex3f(q[j + 1][0] + u[j * 1][0] + u[j + 1][0] * cos(2 * M_PI / 10 * i)*0.2, q[j + 1][1] - 0.1*v[j + 1][1] + v[j + 1][1] * sin(2 * M_PI / 10 * i)*0.2, q[j + 1][2] + u[j+1][2] + u[j+1][2] * cos(2 * M_PI / 10 * i)*0.2);
		}
	}
	glEnd();
	//Center Rail which start point in (q[j][0],q[j][1]-v[j][q],q[j][2])
	glBegin(GL_QUAD_STRIP);
	for (int j = 0; j < (numberPoints*NUMBER_SEGMENTS) - 300; j++){
		for (int i = 0; i <= 10; i++){
			glColor3f(0.96, 0.0, 0.96);
			glVertex3f(q[j][0] + u[j][0] * cos(2 * M_PI / 10 * i)*0.2, q[j][1] - v[j][1] + v[j][1] * sin(2 * M_PI / 10 * i)*0.2, q[j][2] + u[j][2] * cos(2 * M_PI / 10 * i)*0.2);
			glVertex3f(q[j + 1][0] + u[j + 1][0] * cos(2 * M_PI / 10 * i)*0.2, q[j + 1][1] - v[j + 1][1] + v[j + 1][1] * sin(2 * M_PI / 10 * i)*0.2, q[j + 1][2] + u[j + 1][2] * cos(2 * M_PI / 10 * i)*0.2);
		}
	}
	glEnd();
}
/*draw the connectors between two rails*/
void drawConnectors(){
	for (int j = 0; j < (numberPoints*NUMBER_SEGMENTS) - 300; j += NUMBER_SEGMENTS){
		/*connect between left rail and center rail*/
		glBegin(GL_QUAD_STRIP);
		for (int i = 0; i <= 10; i++){
			glColor3f(0.96, 0.0, 0.96);
			glVertex3f(q[j][0] - u[j][0] + u[j][0] * cos(2 * M_PI / 10 * i)*0.2, q[j][1] - 0.1*v[j][1] + v[j][1] * sin(2 * M_PI / 10 * i)*0.2, q[j][2] - u[j][2] + u[j][2] * cos(2 * M_PI / 10 * i)*0.2);
			glVertex3f(q[j][0] + u[j][0] * cos(2 * M_PI / 10 * i)*0.2, q[j][1] - v[j][1] + v[j][1] * sin(2 * M_PI / 10 * i)*0.2, q[j][2] + u[j][2] * cos(2 * M_PI / 10 * i)*0.2);
		}
		glEnd();
		/*connect between right rail and center rail*/
		glBegin(GL_QUAD_STRIP);
		for (int i = 0; i <= 10; i++){
			glColor3f(0.96, 0.0, 0.96);
			glVertex3f(q[j][0] + u[j][0] + u[j][0] * cos(2 * M_PI / 10 * i)*0.2, q[j][1] - 0.1*v[j][1] + v[j][1] * sin(2 * M_PI / 10 * i)*0.2, q[j][2] + u[j][2] + u[j][2] * cos(2 * M_PI / 10 * i)*0.2);
			glVertex3f(q[j][0] + u[j][0] * cos(2 * M_PI / 10 * i)*0.2, q[j][1] - v[j][1] + v[j][1] * sin(2 * M_PI / 10 * i)*0.2, q[j][2] + u[j][2] * cos(2 * M_PI / 10 * i)*0.2);
		}
		glEnd();
	}
}
/*draw the columns to the ground*/
void drawColumns(){
	for (int j = 0; j < (numberPoints*NUMBER_SEGMENTS) - 300; j += NUMBER_SEGMENTS){
		/*column from left rail to ground*/
		glBegin(GL_QUAD_STRIP);
		for (int i = 0; i <= 10; i++){
			glColor3f(0.96, 0.0, 0.96);
			glVertex3f(q[j][0] - u[j][0] + u[j][0] * cos(2 * M_PI / 10 * i)*0.05, q[j][1] - 0.1*v[j][1] + v[j][1] * sin(2 * M_PI / 10 * i)*0.05, q[j][2] - u[j][2] + u[j][2] * cos(2 * M_PI / 10 * i)*0.05);
			glVertex3f(q[j][0] - u[j][0] + u[j][0] * cos(2 * M_PI / 10 * i)*0.05, -40, q[j][2] - u[j][2] + u[j][2] * cos(2 * M_PI / 10 * i)*0.05);
		}
		glEnd();
		/*column from right rail to ground*/
		glBegin(GL_QUAD_STRIP);
		for (int i = 0; i <= 10; i++){
			glColor3f(0.96, 0.0, 0.96);
			glVertex3f(q[j][0] + u[j][0] + u[j][0] * cos(2 * M_PI / 10 * i)*0.05, q[j][1] - 0.1*v[j][1] + v[j][1] * sin(2 * M_PI / 10 * i)*0.05, q[j][2] + u[j][2] + u[j][2] * cos(2 * M_PI / 10 * i)*0.05);
			glVertex3f(q[j][0] + u[j][0] + u[j][0] * cos(2 * M_PI / 10 * i)*0.05, -40, q[j][2] + u[j][2] + u[j][2] * cos(2 * M_PI / 10 * i)*0.05);
		}
		glEnd();
	}
}
/*draw the sky*/
void drawSkybox(){
	// Draw the bottom of the skybox.
	glBegin(GL_QUAD_STRIP);
	for (int i = 0; i <= 360; i++){
		// dark blue.
		glColor3f(0.15, 0.15, 0.55);
		glVertex3f(180*cos(i), 100, 180*sin(i));
		// light blue.
		glColor3f(0.75, 0.85, 0.85);
		glVertex3f(180*cos(i), -100, 180*sin(i));
	}
	glEnd();
	// Draw the top of the skybox.
	glBegin(GL_TRIANGLE_FAN);
	glColor3f(0.0, 0.0, 0.55);
	glVertex3f(0, 100, 0);
	for (int i = 0; i <= 360; i++){
		glVertex3f(180*cos(i), 100, 180*sin(i));
	}
	glEnd();
}

/* The display calllback function for the program. This will be called for each frame of the simulation. */
void myDisplay(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Look at the origin and rotate around the roller coaster while maintaining a distance of 50.
	glLoadIdentity();
	if (cameraMode == 1){
		gluLookAt(cos(phi)*100.0, 50, sin(phi)*100.0,
			0.0, 0.0, 0.0,
			0.0, 1.0, 0.0);
	}
	// Use the up vector as the v vector.
	else{
		gluLookAt(q[currentPosition][0] + up[currentPosition][0], q[currentPosition][1] + up[currentPosition][1], q[currentPosition][2] + up[currentPosition][2],
			q[currentPosition][0] + dq[currentPosition][0], q[currentPosition][1] + dq[currentPosition][1], q[currentPosition][2] + dq[currentPosition][2],
			v[currentPosition][0], v[currentPosition][1], v[currentPosition][2]);
	}
	glCallList(scene);

	glutSwapBuffers();
}

/* The timer call back for the game over last as long as the time wait macro
* is specified for. It does not have anything to update other than the timer.
*/
void myTimer(int value){
	/*
	* Timer callback for screen between the levels.
	*/
	phi = (phi + 0.005);

	// Update the velocity based on the height.
	velocity = (int)((1.0 / 2.0)*((sqrt(2.0*((10000.0 / 10.0) - (9.81*(q[currentPosition][1]+40)))))));
	currentPosition = (currentPosition + velocity) % (NUMBER_SEGMENTS*numberPoints - (3 * NUMBER_SEGMENTS));

	glutPostRedisplay();

	glutTimerFunc(33, myTimer, 0);

}
/* The reshape function for this program sets up a new window based on the specified
aspect ratio. It also sets up the new viewing coordinates for rendering.*/
void myReshape(int w, int h)
{
	/*
	*	reshape callback function; the upper and lower boundaries of the
	*	window are at 100.0 and 0.0, respectively; the aspect ratio is
	*  determined by the aspect ratio of the viewport
	*/

	xMax = 100.0*w / h;
	yMax = 100.0;

	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70, (GLdouble)2, 0.1, 300.0);

	glMatrixMode(GL_MODELVIEW);
}

/* We can use the arrow keys to move around the roller coaster scene. */
void
keyPress(int key, int x, int y)
{
	/*
	*	this function is called when a special key is pressed; we are
	*	interested in the cursor keys only
	*/
}

// Takes keyboard input and acts accordingly based on the key pressed.
void myKey(unsigned char key, int x, int y)
{
	if (key == 'q' || key == 'Q'){
		free(q);
		free(dq);
		free(ddq);
		free(u);
		free(v);
		free(n);
		exit(0);
	}
	// Switch camera positions by pressing r.
	if(key== 'r'||key=='R')
		cameraMode = -1 * cameraMode;
}

/* We can use the arrow keys to move around the roller coaster scene. This will help with develop
of the shape of the roller coaster and also with viewing the coaster upon completion. This will
check for a release. */
void keyRelease(int key, int x, int y)
{
	/*
	*	this function is called when a special key is released; we are
	*	interested in the cursor keys only
	*/
}