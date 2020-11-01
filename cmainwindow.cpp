#ifndef ULAPI
#define ULAPI
#endif

#include "hal.h"
#include "cmainwindow.h"
#include "ui_cmainwindow.h"
#include <QStringList>

//hal realtime layer
typedef struct {
hal_float_t *pin;
} float_data_t;

typedef struct {
hal_bit_t *pin;
} bit_data_t;

int comp_id = hal_init("gui");

float_data_t *float_data_0 = (float_data_t*)hal_malloc(sizeof(float_data_t));
int retval0 = hal_pin_float_new("J2_POS_CMD",HAL_OUT,&(float_data_0->pin),comp_id);

float_data_t *float_data_1 = (float_data_t*)hal_malloc(sizeof(float_data_t));
int retval1 = hal_pin_float_new("J3_POS_CMD",HAL_OUT,&(float_data_1->pin),comp_id);

//example to set up bit data
//bit_data_t *bit_data_0 = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
//int retvalxx = hal_pin_bit_new("bitpin",HAL_OUT,&(bit_data_0->pin),comp_id);

int ready = hal_ready(comp_id);
//end hal realtime layer

CMainWindow::CMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CMainWindow)
{
    ui->setupUi(this);
    m_3d_widget = new C3DWidget(this);
    ui->gridLayout->addWidget(m_3d_widget,0,0);

    //start ethercat..
    system("halcmd \-f robot/Ethercat/ethercat.hal");
    system("halcmd start");
}

CMainWindow::~CMainWindow()
{
    hal_exit(comp_id);
    system("halrun -U"); //this is really important.
    delete ui;
}

void CMainWindow::on_pushButton_open_stepfile_pressed()
{
    m_3d_widget->open_stepfile();
    on_horizontalSlider_J1_valueChanged(0); //just to do a initial calculation to update gui toolpos values.
    update_endeffector_pos();
}

void CMainWindow::on_horizontalSlider_base_valueChanged(int value)
{
    m_3d_widget->update_joint_slider(0, value); //This value has to be adapted to the KDL kinematics frame.
    update_endeffector_pos();
}

void CMainWindow::on_horizontalSlider_J1_valueChanged(int value)
{
    m_3d_widget->update_joint_slider(1, value);
    update_endeffector_pos();
}

void CMainWindow::on_horizontalSlider_J2_valueChanged(int value)
{
    m_3d_widget->update_joint_slider(2, value);
    update_endeffector_pos();
}

void CMainWindow::on_horizontalSlider_J3_valueChanged(int value)
{
    m_3d_widget->update_joint_slider(3, value);
    update_endeffector_pos();
}

void CMainWindow::on_horizontalSlider_J4_valueChanged(int value)
{
    m_3d_widget->update_joint_slider(4, value);
    update_endeffector_pos();
}

void CMainWindow::on_horizontalSlider_J5_valueChanged(int value)
{
    m_3d_widget->update_joint_slider(5, value);
    update_endeffector_pos();
}

void CMainWindow::on_horizontalSlider_J6_valueChanged(int value)
{
    m_3d_widget->update_joint_slider(6, value);
    update_endeffector_pos();
}

void CMainWindow::on_pushButton_erase_all_pressed()
{
    m_3d_widget->erase_all();
}

void CMainWindow::on_pushButton_xmin_pressed()
{
    m_3d_widget->update_joint_slider(100, ui->spinBox_stepsize->value()*-1);
    update_sliders();
    update_endeffector_pos();
    //Mention thisis doing a whole kinematics recalculation instead of only moving the gui slider pos a little bit.
    //For now i am oke with this.
}

void CMainWindow::on_pushButton_xplus_pressed()
{
    m_3d_widget->update_joint_slider(100, ui->spinBox_stepsize->value());
    update_sliders();
    update_endeffector_pos();
}

void CMainWindow::on_pushButton_ymin_pressed()
{
    m_3d_widget->update_joint_slider(101, ui->spinBox_stepsize->value()*-1);
    update_sliders();
    update_endeffector_pos();
}

void CMainWindow::on_pushButton_yplus_pressed()
{
    m_3d_widget->update_joint_slider(101, ui->spinBox_stepsize->value());
    update_sliders();
    update_endeffector_pos();
}

void CMainWindow::on_pushButton_zmin_pressed()
{
    m_3d_widget->update_joint_slider(102, ui->spinBox_stepsize->value()*-1);
    update_sliders();
    update_endeffector_pos();
}

void CMainWindow::on_pushButton_zplus_pressed()
{
    m_3d_widget->update_joint_slider(102, ui->spinBox_stepsize->value());
    update_sliders();
    update_endeffector_pos();
}

void CMainWindow::on_pushButton_reset_joints_pressed()
{
    m_3d_widget->update_joint_slider(1, 0);
    m_3d_widget->update_joint_slider(2, 0);
    m_3d_widget->update_joint_slider(3, 0);
    m_3d_widget->update_joint_slider(4, 0);
    m_3d_widget->update_joint_slider(5, 0);
    m_3d_widget->update_joint_slider(6, 0);
    update_sliders();
    update_endeffector_pos();
}

void CMainWindow::update_sliders(){

    ui->horizontalSlider_J1->setValue(deg_joint1);
    ui->horizontalSlider_J2->setValue(deg_joint2);
    ui->horizontalSlider_J3->setValue(deg_joint3);
    ui->horizontalSlider_J4->setValue(deg_joint4);
    ui->horizontalSlider_J5->setValue(deg_joint5);
    ui->horizontalSlider_J6->setValue(deg_joint6);
}

void CMainWindow::update_endeffector_pos(){

    //Todo add : euler_x,euler_y,euler_z
    QStringList strList;
    strList << "X" << QString::number(endeffector_x,'f',2) << ", Y" << QString::number(endeffector_y,'f',2) << ", Z" << QString::number(endeffector_z,'f',2);
    QString str = strList.join("");
    ui->lineEdit_endeffector_pos->setText(str);

    //Initialize the min and max value's of the robot sliders has to be done.
    //See kinematics.cpp
    ui->lineEdit_Floor->setText(QString::number(deg_base,'f',2));
    ui->lineEdit_J1->setText(QString::number(deg_joint1,'f',2));
    ui->lineEdit_J2->setText(QString::number(deg_joint2,'f',2));
    ui->lineEdit_J3->setText(QString::number(deg_joint3,'f',2));
    ui->lineEdit_J4->setText(QString::number(deg_joint4,'f',2));
    ui->lineEdit_J5->setText(QString::number(deg_joint5,'f',2));
    ui->lineEdit_J6->setText(QString::number(deg_joint6,'f',2));

    //realtime robot position command to "hardware abstract layer", HAL.
    *((hal_float_t *) (float_data_0->pin)) = deg_joint2;
    *((hal_float_t *) (float_data_1->pin)) = deg_joint3;
}

void CMainWindow::on_pushButton_halshow_pressed()
{
    system("halshow &");
}
















