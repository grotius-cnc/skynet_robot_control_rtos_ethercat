#ifndef CMAINWINDOW_H
#define CMAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>

#include <TopTools_HSequenceOfShape.hxx>

#include "c3dwidget.h"
#include "kinematics.h"

namespace Ui {
class CMainWindow;
}

class CMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit CMainWindow(QWidget *parent = 0);
    ~CMainWindow();
    C3DWidget* m_3d_widget;
private slots:

    void on_pushButton_erase_all_pressed();

    void on_pushButton_open_stepfile_pressed();

    void on_horizontalSlider_J1_valueChanged(int value);

    void on_horizontalSlider_J2_valueChanged(int value);

    void on_horizontalSlider_J3_valueChanged(int value);

    void on_horizontalSlider_J4_valueChanged(int value);

    void on_horizontalSlider_J5_valueChanged(int value);

    void on_horizontalSlider_J6_valueChanged(int value);

    void on_horizontalSlider_base_valueChanged(int value);

    void update_sliders();

    void update_endeffector_pos();

    void on_pushButton_xmin_pressed();

    void on_pushButton_xplus_pressed();

    void on_pushButton_ymin_pressed();

    void on_pushButton_yplus_pressed();

    void on_pushButton_zmin_pressed();

    void on_pushButton_zplus_pressed();

    void on_pushButton_reset_joints_pressed();

    void on_pushButton_halshow_pressed();

private:
    Ui::CMainWindow *ui;
};

#endif // CMAINWINDOW_H
