#include "dlg_color_palette.h"
#include "ui_dlg_color_palette.h"

dlg_color_palette::dlg_color_palette(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::dlg_color_palette)
{
    ui->setupUi(this);
}

dlg_color_palette::~dlg_color_palette()
{
    delete ui;
}

int dlg_color_palette::axis() {
    return ui->comboBox_Axis->currentIndex();
}

int dlg_color_palette::mode() {
    return ui->comboBox_Mode->currentIndex();
}

void dlg_color_palette::initUi(int axis,int mode) {
    ui->comboBox_Axis->setCurrentIndex(axis);
    ui->comboBox_Mode->setCurrentIndex(mode);
}
