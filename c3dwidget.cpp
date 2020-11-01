/* ====================================================
#   Copyright (C)2019 Li Jianbin All rights reserved.
#
#   Author        : Li Jianbin
#   Email         : lijianbinmail@163.com
#   File Name     : c3dwidget.cpp
#   Last Modified : 2019-04-21 15:00
#   Describe      : 3D Widget
#
# ====================================================*/

#include "c3dwidget.h"
#include "makebottle.h"
#include <Geom_Line.hxx>
#include <Geom_CartesianPoint.hxx>
#include <TPrsStd_AISPresentation.hxx>
#include <TDF_Label.hxx>
#include <AIS_Selection.hxx>

//https://github.com/lvk88/OccTutorial/blob/master/OtherExamples/runners/moveAndRotateAssem.cpp
#include "BRepTools_ReShape.hxx"
#include "BRepTools.hxx"
#include "TopTools.hxx"
#include "STEPControl_Writer.hxx"

#include "BRepBuilderAPI_Transform.hxx"
#include "gp_Trsf.hxx"
#include "TopoDS.hxx"
#include "TopoDS_Solid.hxx"
#include "TopExp.hxx"
#include "TopTools_IndexedMapOfShape.hxx"
#include "TopExp_Explorer.hxx"
#include "TopoDS_Compound.hxx"
#include "STEPControl_Reader.hxx"

#include "STEPCAFControl_Reader.hxx"

#include <gce_MakeRotation.hxx>

#include <kinematics.h>

TopoDS_Shape ashape,shape0,shape1,shape2,shape3,shape4,shape5,shape6,shape7;
TopoDS_Solid solid0,solid1,solid2,solid3,solid4,solid5,solid6,solid7;
Handle(AIS_Shape) ais_shape0,ais_shape1,ais_shape2,ais_shape3,ais_shape4,ais_shape5,ais_shape6,ais_shape7;

double deg_base,deg_joint1,deg_joint2,deg_joint3,deg_joint4,deg_joint5,deg_joint6,trans_x,trans_y,trans_z,endeffector_x,endeffector_y,endeffector_z,euler_x,euler_y,euler_z;
gp_Trsf myTrsf0,myTrsf1,myTrsf2,myTrsf3,myTrsf4,myTrsf5,myTrsf6;
FWKIN_DEG IN,OUT;

//robot part nr's extracted from step file:
int base_nr=1;
int joint1_nr=2;

C3DWidget::C3DWidget(QWidget *parent) : QGLWidget(parent)
{
    setBackgroundRole( QPalette::NoRole );
    setMouseTracking( true );
}

void C3DWidget::erase_selected(){

    m_context->EraseSelected(1);
}

void C3DWidget::erase_all(){

    m_context->EraseAll(1);
}

void C3DWidget::open_stepfile(){

    STEPControl_Reader Reader0, Reader1, Reader2, Reader3, Reader4, Reader5, Reader6;
    Reader0.ReadFile("robot/kuka_base.step");
    Reader1.ReadFile("robot/kuka_joint_1.step");
    Reader2.ReadFile("robot/kuka_joint_2.step");
    Reader3.ReadFile("robot/kuka_joint_3.step");
    Reader4.ReadFile("robot/kuka_joint_4.step");
    Reader5.ReadFile("robot/kuka_joint_5.step");
    Reader6.ReadFile("robot/kuka_joint_6.step");

    std::cout << Reader0.TransferRoots() << " Reader0 roots transferred." << std::endl;
    std::cout << Reader1.TransferRoots() << " Reader1 roots transferred." << std::endl;
    std::cout << Reader2.TransferRoots() << " Reader2 roots transferred." << std::endl;
    std::cout << Reader3.TransferRoots() << " Reader3 roots transferred." << std::endl;
    std::cout << Reader4.TransferRoots() << " Reader4 roots transferred." << std::endl;
    std::cout << Reader5.TransferRoots() << " Reader5 roots transferred." << std::endl;
    std::cout << Reader6.TransferRoots() << " Reader6 roots transferred." << std::endl;

    shape0=Reader0.OneShape();
    shape1=Reader1.OneShape();
    shape2=Reader2.OneShape();
    shape3=Reader3.OneShape();
    shape4=Reader4.OneShape();
    shape5=Reader5.OneShape();
    shape6=Reader6.OneShape();

    ais_shape0=new AIS_Shape(shape0); //floor
    ais_shape1=new AIS_Shape(shape1); //robot base
    ais_shape2=new AIS_Shape(shape2); //robot base
    ais_shape3=new AIS_Shape(shape3); //robot base
    ais_shape4=new AIS_Shape(shape4); //robot base
    ais_shape5=new AIS_Shape(shape5); //robot base
    ais_shape6=new AIS_Shape(shape6); //robot base

    m_context->Display(ais_shape0,Standard_False);
    m_context->Display(ais_shape1,Standard_False);
    m_context->Display(ais_shape2,Standard_False);
    m_context->Display(ais_shape3,Standard_False);
    m_context->Display(ais_shape4,Standard_False);
    m_context->Display(ais_shape5,Standard_False);
    m_context->Display(ais_shape6,Standard_False);

    m_view->FitAll();
}

void C3DWidget::update_joint_slider(int joint, int degrees){

    std::cout<<"rotation in degrees:"<<degrees<<std::endl;

    switch (joint)  {
    case 0:
        deg_base=degrees;
        myTrsf0.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),deg_base * M_PI /180); //gp_Dir rotates around z axis.
        break;
    case 1:
        deg_joint1=degrees;
        myTrsf1.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),deg_joint1 * M_PI /180); //gp_Dir rotates around z axis.
        get_various_positions(); //used to update the mainwindow end effector values.
        break;
    case 2:
        deg_joint2=degrees;
        myTrsf2.SetRotation(gp_Ax1(gp_Pnt(25,0,400),gp_Dir(0,1,0)),deg_joint2 * M_PI /180); //gp_Dir rotates around z axis.
        get_various_positions();
        break;
    case 3:
        deg_joint3=degrees;
        myTrsf3.SetRotation(gp_Ax1(gp_Pnt(25,0,855),gp_Dir(0,1,0)),deg_joint3 * M_PI /180); //gp_Dir rotates around z axis.
        get_various_positions();
        break;
    case 4:
        deg_joint4=degrees;
        myTrsf4.SetRotation(gp_Ax1(gp_Pnt(25,0,890),gp_Dir(1,0,0)),deg_joint4 * M_PI /180); //gp_Dir rotates around z axis.
        get_various_positions();
        break;
    case 5:
        deg_joint5=degrees;
        myTrsf5.SetRotation(gp_Ax1(gp_Pnt(445,0,890),gp_Dir(0,1,0)),deg_joint5 * M_PI /180); //gp_Dir rotates around z axis.
        get_various_positions();
        break;
    case 6:
        deg_joint6=degrees;
        myTrsf6.SetRotation(gp_Ax1(gp_Pnt(445,0,890),gp_Dir(1,0,0)),deg_joint6 * M_PI /180); //gp_Dir rotates around z axis.
        get_various_positions();
        break;
    case 100: //x axis move
        trans_x=degrees; //used as wrapper for lineair value
        //KDL kinematics library, kinematics.h

        //Todo is give the robot the exact xyz target position every xyz move command.
        //At the moment the kinematic offpath tollerence (math presicion) is repeated every time, so offpath tollerence is becoming greater each time.
        //This is quite nice to see happening.

        IN.J1=deg_joint1;
        IN.J2=deg_joint2;
        IN.J3=deg_joint3;
        IN.J4=deg_joint4;
        IN.J5=deg_joint5;
        IN.J6=deg_joint6;
        IN.Xtrans=trans_x;
        IN.Ytrans=trans_y;
        IN.Ztrans=trans_z;
        OUT = kinematics().kinematics_inv(IN);
        myTrsf0.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),deg_base * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf1.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),OUT.J1 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf2.SetRotation(gp_Ax1(gp_Pnt(25,0,400),gp_Dir(0,1,0)),OUT.J2 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf3.SetRotation(gp_Ax1(gp_Pnt(25,0,855),gp_Dir(0,1,0)),OUT.J3 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf4.SetRotation(gp_Ax1(gp_Pnt(25,0,890),gp_Dir(1,0,0)),OUT.J4 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf5.SetRotation(gp_Ax1(gp_Pnt(445,0,890),gp_Dir(0,1,0)),OUT.J5 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf6.SetRotation(gp_Ax1(gp_Pnt(445,0,890),gp_Dir(1,0,0)),OUT.J6 * M_PI /180); //gp_Dir rotates around z axis.

        deg_joint1=OUT.J1;
        deg_joint2=OUT.J2;
        deg_joint3=OUT.J3;
        deg_joint4=OUT.J4;
        deg_joint5=OUT.J5;
        deg_joint6=OUT.J6;

        //user to update mainwindow end effector value's:
        endeffector_x=OUT.Xee;
        endeffector_y=OUT.Yee;
        endeffector_z=OUT.Zee;
        euler_x=OUT.EulerX;
        euler_y=OUT.EulerY;
        euler_z=OUT.EulerZ;

        trans_x=0; trans_y=0; trans_z=0;

        break;
    case 101: //y axis move
        trans_y=degrees; //used as wrapper for lineair value
        //KDL kinematics library, kinematics.h
        IN.J1=deg_joint1;
        IN.J2=deg_joint2;
        IN.J3=deg_joint3;
        IN.J4=deg_joint4;
        IN.J5=deg_joint5;
        IN.J6=deg_joint6;
        IN.Xtrans=trans_x;
        IN.Ytrans=trans_y;
        IN.Ztrans=trans_z;
        OUT = kinematics().kinematics_inv(IN);
        myTrsf0.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),deg_base * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf1.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),OUT.J1 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf2.SetRotation(gp_Ax1(gp_Pnt(25,0,400),gp_Dir(0,1,0)),OUT.J2 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf3.SetRotation(gp_Ax1(gp_Pnt(25,0,855),gp_Dir(0,1,0)),OUT.J3 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf4.SetRotation(gp_Ax1(gp_Pnt(25,0,890),gp_Dir(1,0,0)),OUT.J4 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf5.SetRotation(gp_Ax1(gp_Pnt(445,0,890),gp_Dir(0,1,0)),OUT.J5 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf6.SetRotation(gp_Ax1(gp_Pnt(445,0,890),gp_Dir(1,0,0)),OUT.J6 * M_PI /180); //gp_Dir rotates around z axis.

        deg_joint1=OUT.J1;
        deg_joint2=OUT.J2;
        deg_joint3=OUT.J3;
        deg_joint4=OUT.J4;
        deg_joint5=OUT.J5;
        deg_joint6=OUT.J6;

        //user to update mainwindow end effector value's:
        endeffector_x=OUT.Xee;
        endeffector_y=OUT.Yee;
        endeffector_z=OUT.Zee;
        euler_x=OUT.EulerX;
        euler_y=OUT.EulerY;
        euler_z=OUT.EulerZ;

        trans_x=0; trans_y=0; trans_z=0;

        break;
    case 102: //z axis move
        trans_z=degrees; //used as wrapper for lineair value
        //KDL kinematics library, kinematics.h
        IN.J1=deg_joint1;
        IN.J2=deg_joint2;
        IN.J3=deg_joint3;
        IN.J4=deg_joint4;
        IN.J5=deg_joint5;
        IN.J6=deg_joint6;
        IN.Xtrans=trans_x;
        IN.Ytrans=trans_y;
        IN.Ztrans=trans_z;
        OUT = kinematics().kinematics_inv(IN);
        myTrsf0.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),deg_base * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf1.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),OUT.J1 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf2.SetRotation(gp_Ax1(gp_Pnt(25,0,400),gp_Dir(0,1,0)),OUT.J2 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf3.SetRotation(gp_Ax1(gp_Pnt(25,0,855),gp_Dir(0,1,0)),OUT.J3 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf4.SetRotation(gp_Ax1(gp_Pnt(25,0,890),gp_Dir(1,0,0)),OUT.J4 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf5.SetRotation(gp_Ax1(gp_Pnt(445,0,890),gp_Dir(0,1,0)),OUT.J5 * M_PI /180); //gp_Dir rotates around z axis.
        myTrsf6.SetRotation(gp_Ax1(gp_Pnt(445,0,890),gp_Dir(1,0,0)),OUT.J6 * M_PI /180); //gp_Dir rotates around z axis.

        deg_joint1=OUT.J1;
        deg_joint2=OUT.J2;
        deg_joint3=OUT.J3;
        deg_joint4=OUT.J4;
        deg_joint5=OUT.J5;
        deg_joint6=OUT.J6;

        //user to update mainwindow end effector value's:
        endeffector_x=OUT.Xee;
        endeffector_y=OUT.Yee;
        endeffector_z=OUT.Zee;
        euler_x=OUT.EulerX;
        euler_y=OUT.EulerY;
        euler_z=OUT.EulerZ;

        trans_x=0; trans_y=0; trans_z=0;

        break;
    }

    //forward kinematics matrix multiplication, this is a seperate opencascade forward kinematics calculation to preview the robot parts only.
    ais_shape0->SetLocalTransformation(myTrsf0);
    ais_shape1->SetLocalTransformation(myTrsf0*myTrsf1);
    ais_shape2->SetLocalTransformation((myTrsf0*myTrsf1)*myTrsf2);
    ais_shape3->SetLocalTransformation(((myTrsf0*myTrsf1)*myTrsf2)*myTrsf3);
    ais_shape4->SetLocalTransformation((((myTrsf0*myTrsf1)*myTrsf2)*myTrsf3)*myTrsf4);
    ais_shape5->SetLocalTransformation(((((myTrsf0*myTrsf1)*myTrsf2)*myTrsf3)*myTrsf4)*myTrsf5);
    ais_shape6->SetLocalTransformation((((((myTrsf0*myTrsf1)*myTrsf2)*myTrsf3)*myTrsf4)*myTrsf5)*myTrsf6);
    m_context->CurrentViewer()->Redraw();
}

void C3DWidget::get_various_positions(){

    IN.J1=deg_joint1;
    IN.J2=deg_joint2;
    IN.J3=deg_joint3;
    IN.J4=deg_joint4;
    IN.J5=deg_joint5;
    IN.J6=deg_joint6;
    IN.Xtrans=trans_x;
    IN.Ytrans=trans_y;
    IN.Ztrans=trans_z;
    OUT = kinematics().kinematics_fwd(IN); //forward kinematics only to calulate end-effector xyz pos + euler angles.

    endeffector_x=OUT.Xee;
    endeffector_y=OUT.Yee;
    endeffector_z=OUT.Zee;
    euler_x=OUT.EulerX;
    euler_y=OUT.EulerY;
    euler_z=OUT.EulerZ;
}


void C3DWidget::button_rotate(bool value){

    gp_Trsf myTrsf;
    if(value){
        myTrsf.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,1,0)),10 * M_PI /180);

    } else {
        myTrsf.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,1,0)),-10 * M_PI /180);

    }
    //BRepBuilderAPI_Transform xform1(shape, myTrsf);
    //shape = xform1.Shape();
    //updateAIS(shape,anAIS,m_context);
}

void C3DWidget::updateAIS(TopoDS_Shape aShape, Handle_AIS_Shape &anAIS, Handle_AIS_InteractiveContext ic)
{
    if(aShape.IsNull()) return;
    if (anAIS.IsNull()){
        anAIS=new AIS_Shape(aShape);
        //ic->SetMaterial(anAIS,Graphic3d_NOM_NEON_GNC,0);
        //ic->SetColor(anAIS, Quantity_NOC_BLACK,0);
        ic->SetDisplayMode(anAIS,1,Standard_False); //middle 1 to 0 = wireframe preview
    }
    if (!anAIS->HasPresentation()){
        ic->Display(anAIS, 1,0,false,false);
    }
    else
    {
        anAIS->Set(aShape);
        ic->Deactivate(anAIS);
        ic->Redisplay(anAIS,true,true);
    }
    //ic->Update(anAIS,true);
    //ic->CurrentViewer()->Redraw();
    ic->Display(anAIS,1);
}

void C3DWidget::rotate_selected(){


}

void C3DWidget::rotate_line(){

    TopoDS_Shape lsWall= BRepPrimAPI_MakeBox(100,1,1).Shape();

    int r=45;

    gp_Trsf myTrsf;
    myTrsf.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,1,0)),r * M_PI /180);

    BRepBuilderAPI_Transform xform1(lsWall, myTrsf);
    lsWall = xform1.Shape();

    Handle(AIS_Shape) t_ais_box = new AIS_Shape(lsWall);

    m_context->Display(t_ais_box, Standard_True);
    m_view->FitAll();
}

void C3DWidget::translate_line(){

    TopoDS_Shape lsWall= BRepPrimAPI_MakeBox(100,1,1).Shape();

    int offset=25;

    gp_Trsf myTrsf;
    myTrsf.SetTranslation(gp_Pnt(0,0,0),gp_Pnt(0,offset,0));

    BRepBuilderAPI_Transform xform1(lsWall, myTrsf);
    lsWall=xform1.Shape();

    Handle(AIS_Shape) t_ais_box = new AIS_Shape(lsWall);

    m_context->Display(t_ais_box,1);
    m_view->FitAll();
}

void C3DWidget::make_cube(Standard_Real _dx, Standard_Real _dy, Standard_Real _dz)
{
    TopoDS_Shape t_topo_box = BRepPrimAPI_MakeBox(_dx, _dy, _dz).Shape();
    Handle(AIS_Shape) t_ais_box = new AIS_Shape(t_topo_box);
    m_context->Display(t_ais_box, Standard_True);
    m_view->FitAll();
}

void C3DWidget::make_cylinder(Standard_Real _R, Standard_Real _H)
{
    TopoDS_Shape t_topo_cylinder = BRepPrimAPI_MakeCylinder(_R , _H).Shape();
    Handle(AIS_Shape) t_ais_cylinder = new AIS_Shape(t_topo_cylinder);
    m_context->Display(t_ais_cylinder, Standard_True);
    m_view->FitAll();
}

void C3DWidget::make_sphere(Standard_Real _R)
{
    TopoDS_Shape t_topo_sphere = BRepPrimAPI_MakeSphere(_R).Shape();
    Handle(AIS_Shape) t_ais_sphere = new AIS_Shape(t_topo_sphere);
    m_context->Display(t_ais_sphere, Standard_True);
    m_view->FitAll();
}

void C3DWidget::make_cone(Standard_Real _R1, Standard_Real _R2, Standard_Real _H)
{
    TopoDS_Shape t_topo_cone = BRepPrimAPI_MakeCone(_R1,_R2,_H).Shape();
    Handle(AIS_Shape) t_ais_cone = new AIS_Shape(t_topo_cone);
    m_context->Display(t_ais_cone, Standard_True);
    m_view->FitAll();
}

void C3DWidget::make_torus(Standard_Real _R1, Standard_Real _R2)
{
    TopoDS_Shape t_topo_torus = BRepPrimAPI_MakeTorus(_R1 ,_R2).Shape();
    Handle(AIS_Shape) t_ais_torus = new AIS_Shape(t_topo_torus);
    m_context->Display(t_ais_torus, Standard_True);
    m_view->FitAll();
}

void C3DWidget::m_initialize_context()
{
    if (m_context.IsNull())
    {

        Handle(Aspect_DisplayConnection) m_display_donnection = new Aspect_DisplayConnection();

        if (m_graphic_driver.IsNull())
        {
            m_graphic_driver = new OpenGl_GraphicDriver(m_display_donnection);
        }

        WId window_handle = (WId) winId();
#ifdef _WIN32
        Handle(WNT_Window) wind = new WNT_Window((Aspect_Handle) window_handle);
#else
        Handle(Xw_Window) wind = new Xw_Window(m_display_donnection, (Window) window_handle);
#endif

        m_viewer = new V3d_Viewer(m_graphic_driver);

        m_view = m_viewer->CreateView();
        m_view->SetWindow(wind);

        if (!wind->IsMapped())
        {
            wind->Map();
        }
        m_context = new AIS_InteractiveContext(m_viewer);

        m_viewer->SetDefaultLights();
        m_viewer->SetLightOn();

        m_view->SetBackgroundColor(Quantity_NOC_GRAY49);
        m_view->MustBeResized();

        m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_GOLD, 0.08, V3d_ZBUFFER);

        m_context->SetDisplayMode(AIS_Shaded, Standard_True);

        Handle(Prs3d_Drawer) t_hilight_style = m_context->HighlightStyle();
        t_hilight_style->SetMethod(Aspect_TOHM_COLOR);
        t_hilight_style->SetColor(Quantity_NOC_LIGHTYELLOW);
        t_hilight_style->SetDisplayMode(1);
        t_hilight_style->SetTransparency(0.2f);

        Handle(Prs3d_Drawer) t_select_style = m_context->SelectionStyle();
        t_select_style->SetMethod(Aspect_TOHM_COLOR);
        t_select_style->SetColor(Quantity_NOC_LIGHTSEAGREEN);
        t_select_style->SetDisplayMode(1);
        t_select_style->SetTransparency(0.4f);

        m_view->SetZoom(100);

        m_viewer->SetRectangularGridValues(0,0,1,1,0);
        m_viewer->SetRectangularGridGraphicValues(2.01,2.01,0);
        m_viewer->ActivateGrid(Aspect_GT_Rectangular,Aspect_GDM_Lines);
    }
}

void C3DWidget::paintEvent(QPaintEvent *)
{
    if (m_context.IsNull())
    {
        m_initialize_context();
    }
    m_view->Redraw();
}

void C3DWidget::resizeEvent(QResizeEvent *)
{
    if( !m_view.IsNull() )
    {
        m_view->MustBeResized();
    }
}

void C3DWidget::mousePressEvent(QMouseEvent *event)
{
    if((event->buttons()&Qt::LeftButton) && (event->buttons()&Qt::RightButton))
    {
        m_x_max=event->x();
        m_y_max=event->y();
    }
    else if(event->buttons()&Qt::LeftButton)
    {
        m_context->MoveTo(event->pos().x(),event->pos().y(),m_view,Standard_True);

        AIS_StatusOfPick t_pick_status = AIS_SOP_NothingSelected;
        if(qApp->keyboardModifiers()==Qt::ControlModifier)
        {
            t_pick_status = m_context->ShiftSelect(true);
        }
        else
        {
            t_pick_status = m_context->Select(true);
        }
    }
    else if(event->buttons()&Qt::MidButton)
    {
        m_x_max=event->x();
        m_y_max=event->y();
        m_view->StartRotation(event->x(),event->y());
    }



}

void C3DWidget::mouseReleaseEvent(QMouseEvent *event)
{
    m_context->MoveTo(event->pos().x(),event->pos().y(),m_view,Standard_True);
}

void C3DWidget::mouseMoveEvent(QMouseEvent *event)
{
    if((event->buttons()&Qt::LeftButton) && (event->buttons()&Qt::RightButton))
    {
        m_view->Pan(event->pos().x()-m_x_max,m_y_max-event->pos().y());
        m_x_max=event->x();
        m_y_max=event->y();
    }
    else if(event->buttons()&Qt::MidButton)
    {
        if(qApp->keyboardModifiers()==Qt::ShiftModifier)
        {
            m_view->Pan(event->pos().x()-m_x_max,m_y_max-event->pos().y());
            m_x_max=event->x();
            m_y_max=event->y();
        }
        else
        {
            m_view->Rotation(event->x(),event->y());
        }
    }
    else
    {
        m_context->MoveTo(event->pos().x(),event->pos().y(),m_view,Standard_True);
    }
}

void C3DWidget::wheelEvent(QWheelEvent *event)
{
    m_view->StartZoomAtPoint(event->pos().x(),event->pos().y());
    m_view->ZoomAtPoint(0, 0, event->angleDelta().y(), 0);
}
