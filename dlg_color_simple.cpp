#include "dlg_color_simple.h"
#include "ui_dlg_color_simple.h"

dlg_color_simple::dlg_color_simple(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::dlg_color_simple)
{
    ui->setupUi(this);
    initUi(128, 128, 128);

    connect(ui->btnRand, SIGNAL(clicked()), this, SLOT(randomButtonPressed()));
}

dlg_color_simple::~dlg_color_simple()
{
    delete ui;
}

int dlg_color_simple::r() {
    return ui->spinBox_R->value();
}
int dlg_color_simple::g() {
    return ui->spinBox_G->value();
}
int dlg_color_simple::b() {
    return ui->spinBox_B->value();
}
void dlg_color_simple::initUi(int r, int g, int b) {
    ui->spinBox_R->setValue(r);
    ui->spinBox_G->setValue(g);
    ui->spinBox_B->setValue(b);
}
void dlg_color_simple::randomButtonPressed() {
    printf("Random button was pressed\n");
    int red = rand() % 255, green = rand() % 255, blue = rand() % 255;
    //255 * (1024 * rand() / (RAND_MAX + 1.0f));
    initUi(red, green, blue);
}