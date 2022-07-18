#ifndef DLG_COLOR_SIMPLE_H
#define DLG_COLOR_SIMPLE_H

#include <QDialog>

namespace Ui {
class dlg_color_simple;
}

class dlg_color_simple : public QDialog
{
    Q_OBJECT

public:
    explicit dlg_color_simple(QWidget *parent = nullptr);
    ~dlg_color_simple();
    int r();
    int g();
    int b();
    void initUi(int r,int g,int b);
public Q_SLOTS:
    void randomButtonPressed();
private:
    Ui::dlg_color_simple *ui;
};

#endif // DLG_COLOR_SIMPLE_H
