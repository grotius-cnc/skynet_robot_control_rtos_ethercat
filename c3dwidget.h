/* ====================================================
#   Copyright (C)2019 Li Jianbin All rights reserved.
#
#   Author        : Li Jianbin
#   Email         : lijianbinmail@163.com
#   File Name     : c3dwidget.h
#   Last Modified : 2019-04-21 15:00
#   Describe      : 3D Widget
#
# ====================================================*/

#ifndef C3DWIDGET_H
#define C3DWIDGET_H

#include <qtextstream.h>

#include <QWidget>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>

#include <QApplication>

#include <AIS_InteractiveContext.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <V3d_View.hxx>
#include <Aspect_Handle.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Graphic3d_GraphicDriver.hxx>

#include <QGLWidget>
#ifdef _WIN32
#include <WNT_Window.hxx>
#else
#undef None
#include <Xw_Window.hxx>
#endif

#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>

#include <BRepPrimAPI_MakeBox.hxx>
#include <AIS_Shape.hxx>

// Project Include
//#include "cmodel.h"

extern double deg_base,deg_joint1,deg_joint2,deg_joint3,deg_joint4,deg_joint5,deg_joint6,trans_x,trans_y,trans_z,endeffector_x,endeffector_y,endeffector_z,euler_x,euler_y,euler_z;

class C3DWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit C3DWidget(QWidget *parent = nullptr);

    void updateAIS(TopoDS_Shape aShape, Handle_AIS_Shape &anAIS, Handle_AIS_InteractiveContext ic);
    void button_rotate(bool value);
    void erase_all();
    void erase_selected();
    void rotate_selected();
    void rotate_line();
    void translate_line();
    void open_stepfile();
    void update_joint_slider(int joint, int degrees);
    void get_various_positions();

    void make_cube(Standard_Real _dx = 1.0, Standard_Real _dy = 1.0, Standard_Real _dz = 1.0);
    void make_cylinder(Standard_Real _R = 0.5,  Standard_Real _H = 2.0);
    void make_sphere(Standard_Real _R = 1.0);
    void make_cone(Standard_Real _R1 = 1.0, Standard_Real _R2 = 0.0, Standard_Real _H = 2.0);
    void make_torus(Standard_Real _R1 =2.0, Standard_Real _R2 = 0.5);

    Handle(AIS_InteractiveContext) m_get_context(){return m_context;}
    Handle(V3d_View)  m_get_view(){return m_view;}
private:
    void m_initialize_context();
    Handle(AIS_InteractiveContext) m_context;
    Handle(V3d_Viewer) m_viewer;
    Handle(V3d_View) m_view;
    Handle(Graphic3d_GraphicDriver) m_graphic_driver;
protected:
    void paintEvent(QPaintEvent *);
    void resizeEvent(QResizeEvent *);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

protected:
    enum CurrentAction3d
    {
        CurAction3d_Nothing,
        CurAction3d_DynamicPanning,
        CurAction3d_DynamicZooming,
        CurAction3d_DynamicRotation
    };
private:
    Standard_Integer m_x_max;
    Standard_Integer m_y_max;
    CurrentAction3d m_current_mode;

signals:

public slots:
};

#endif // C3DWIDGET_H
