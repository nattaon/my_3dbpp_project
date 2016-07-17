#ifndef GLWIDGET_H
#define GLWIDGET_H
#include "../class_library/mycolor.h"

#include <QtOpenGL/QGLWidget>
#include <QMouseEvent>

class GLWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLWidget(QWidget *parent=0);

    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);

	//automatic enable function
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);


    void SetInitialBin(int binW, int binH,int binD);
	void ResetWindow();
	void SetShowBinpacking(int n,int binW, int binH,int binD, int *w, int *h, int *d, int *x, int *y, int *z);
	void IncreaseShowNumber();
	void DecreaseShowNumber();

public slots:
	// slots for xyz-rotation slider
	void setXRotation(int angle);
	void setYRotation(int angle);
	void setZRotation(int angle);

	void setXTranslation(int value);
	void setYTranslation(int value);

signals:
	// signaling rotation from mouse movement
	//void xRotationChanged(int angle);
	//void yRotationChanged(int angle);
	//void zRotationChanged(int angle);

private:
	//void draw();

	int xRot;
	int yRot;
	int zRot;
	float xMove;
	float yMove;
	float zMove;
	float zoom;
	QPoint lastPos;

	MyColor *Color;
	float *color_rgb;


	int total_boxes;
	int bin_w, bin_h, bin_d;
	int *boxes_w, *boxes_h, *boxes_d;
	int *boxes_x, *boxes_y, *boxes_z;
	int axis_length;

	int show_number;

	bool is_show_binpacking;

	void draw_teapot(float size);
	void DrawRectangle(int x, int y, int z, int w, int h, int d);
	void DrawBin();
	void DrawBoxes();
	void DrawAxis(float length);

	void CalcAxisLength();
};

#endif // GLWIDGET_H
