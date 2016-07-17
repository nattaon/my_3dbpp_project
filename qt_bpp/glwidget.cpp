#include "glwidget.h"


#include <GL/glut.h>
#include <iostream>

static GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
static GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };

static GLfloat mat_ambient[] = { 0.7, 0.7, 0.7, 1.0 };
static GLfloat mat_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
static GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat high_shininess[] = { 100.0 };

GLWidget::GLWidget(QWidget *parent):
    QGLWidget(parent)
{
	ResetWindow();

	Color = new MyColor();
}
void GLWidget::initializeGL()
{
	std::cout << "initializeGL" << std::endl;
	//windowsize=500*500
	//glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

	//glClearColor(0.9, 0.9, 0.9, 1);
    //glClearColor(1.0, 1.0, 1.0, 1);//white background
	glClearColor(0.0, 0.0, 0.0, 1);//black background

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);

	//glEnable(GL_DEPTH_TEST);
	//glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glEnable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);

}
void GLWidget::resizeGL(int w, int h)
{
	//be called at first time
	std::cout << "resizeGL w*h:" << w <<"*"<< h <<std::endl;

	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glOrtho(-250.0, 250.0, -250.0, 250.0, -250.0, 250.0);
	glOrtho(-1 * bin_w, bin_w, -1 * bin_h, bin_h, -1 * bin_d*2, bin_d);//left,right,bottom,top,near.far
	//glOrtho(-2, +2, -2, +2, 1.0, 15.0);

	glMatrixMode(GL_MODELVIEW);

}
void GLWidget::paintGL()
{
	//std::cout << "paintGL" << std::endl;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glTranslatef(xMove, yMove, zMove);


	//std::cout << "rotation " << xRot << " " << yRot << " " << zRot << std::endl;

	glRotatef(xRot , 1.0, 0.0, 0.0);
	glRotatef(yRot , 0.0, 1.0, 0.0);
	glRotatef(zRot , 0.0, 0.0, 1.0);

	glScalef(zoom, zoom, zoom);

	DrawAxis(axis_length);
	DrawBin();		
	if (is_show_binpacking)
	{
		DrawBoxes();
		//glFlush();		

	}

	
}


static void qNormalizeAngle(int &angle)
{
	while (angle < 0)
		angle += 360 ;
	while (angle > 360)
		angle -= 360 ;
}


void GLWidget::setXRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != xRot) {
		xRot = angle;
		//emit xRotationChanged(angle);
		updateGL();
	}
}

void GLWidget::setYRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != yRot) {
		yRot = angle;
		//emit yRotationChanged(angle);
		updateGL();
	}
}

void GLWidget::setZRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != zRot) {
		zRot = angle;
		//emit zRotationChanged(angle);
		updateGL();
	}
}
void GLWidget::setXTranslation(int value)
{
	xMove += value;
	std::cout << "xMove:" << xMove << std::endl;
	updateGL();
}
void GLWidget::setYTranslation(int value)
{
	yMove += value;
	std::cout << "yMove:" << yMove << std::endl;
	updateGL();

}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	//react on left click, right click, wheel pressed
	lastPos = event->pos();
	std::cout << lastPos.x() << "," << lastPos.y() << std::endl;
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastPos.x();
	int dy = event->y() - lastPos.y();

	//std::cout << "dx:dy " << dx << ":" << dy << std::endl;

	if (event->buttons() & Qt::LeftButton) 
	{
		setXRotation(xRot + dy);
		setYRotation(yRot + dx);
	}
	else if (event->buttons() & Qt::RightButton) 
	{
		setXRotation(xRot + dy);
		setZRotation(zRot + dx);
	}
	else if (event->buttons() & Qt::MidButton)
	{
		setXTranslation(dx);
		setYTranslation(dy);
	}

	lastPos = event->pos();
}
void GLWidget::wheelEvent(QWheelEvent *event)
{
	
	//mouse wheel up = zoom in
	if (event->delta() == 120)
	{
		zoom += 0.1;
	}
	//mouse wheel up = zoom out
	else if (event->delta() == -120)
	{
		zoom -= 0.1;
	}
	else
	{
		std::cout << "delta value forbidden " << event->delta() << std::endl;
	}
	std::cout << "zoom=" << zoom << std::endl;
	updateGL();
}

void GLWidget::SetInitialBin(int binW, int binH, int binD)
{
	std::cout << "SetInitialBin" << std::endl;

    bin_w = binW;
    bin_h = binH;
    bin_d = binD;

	CalcAxisLength();

}
void GLWidget::ResetWindow()
{
	xRot = 45;
	yRot = 315;
	zRot = 0;

	xMove = 0;
	yMove = 0;
	zMove = 0;

	zoom = 1.0;

	is_show_binpacking = false;
	total_boxes = 0;
	show_number = 0;
	updateGL();
}
void GLWidget::SetShowBinpacking(int n, int binW, int binH, int binD, int *w, int *h, int *d, int *x, int *y, int *z)
{
	is_show_binpacking = true;

	total_boxes = n;
	show_number = n;

	bin_w = binW;
	bin_h = binH;
	bin_d = binD;

	CalcAxisLength();

	boxes_w = w;
	boxes_h = h;
	boxes_d = d;

	boxes_x = x;
	boxes_y = y;
	boxes_z = z;

	/*for (int i = 0; i < n; i++) {
		std::cout
			<< i << ":"
			<< w[i] << " " << h[i] << " " << d[i] << " "
			<< "pos:"
			<< x[i] << " " << y[i] << " " << z[i] << " "
			<< std::endl;
	}*/
	updateGL();

}
void GLWidget::IncreaseShowNumber()
{
	if (show_number <= total_boxes)
	{
		show_number++;
		updateGL();
	}
}
void GLWidget::DecreaseShowNumber()
{
	if (show_number > 0)
	{
		show_number--;
		updateGL();
	}
}
void GLWidget::draw_teapot(float size)
{


}
void GLWidget::DrawRectangle(int x, int y, int z, int w, int h, int d)
{

	glPushMatrix();
	{

		glTranslatef(x, y, z);
		glTranslatef(w*0.5, h*0.5, d*0.5);
		glScalef(w, h, d);

		glutSolidCube(1.0);

	}
	glPopMatrix();
}
void GLWidget::DrawBin()
{
	glPushMatrix();
	{
		glTranslatef(bin_w/2, bin_h/2, bin_d/2);
		glScalef(bin_w, bin_h, bin_d);
		glColor3d(0.5, 0.5, 0.5);
		glutWireCube(1.0);
		//glutSolidCube(1.0);

	}
	glPopMatrix();

}
void GLWidget::DrawBoxes()
{


	//printf("%d (%d,%d,%d)\n", n, W, H, D);
	//color_num = 0;
	
	for (int i = 0; i < show_number; i++)
	{
		color_rgb=Color->GetColorIndex(i);
		glColor3d(color_rgb[0], color_rgb[1], color_rgb[2]);
		//printf("%2d (%2d %2d %2d) : Bin %2d (%2d, %2d, %2d)\n", i,w[i],h[i],d[i],bno[i],x[i],y[i],z[i]);
		DrawRectangle(boxes_x[i], boxes_y[i], boxes_z[i], boxes_w[i], boxes_h[i], boxes_d[i]);
	}


}
void GLWidget::DrawAxis(float length)
{
	glPushMatrix();
	{
		glLineWidth(2.0);
		glBegin(GL_LINES);
		glColor3d(1.0, 0.0, 0.0);//red
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(length, 0.0, 0.0);

		glColor3d(0.0, 1.0, 0.0);//green
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(0.0, length, 0.0);

		glColor3d(0.0, 0.0, 1.0);//blue
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(0.0, 0.0, length);
		glEnd();
		glLineWidth(1.0);
	}
	glPopMatrix();
}

void GLWidget::CalcAxisLength()
{
	if (bin_w > bin_h)
		axis_length = bin_w;
	else 
		axis_length = bin_h;

	if (bin_d > axis_length)
		axis_length = bin_d;

	axis_length += axis_length / 2;

}
