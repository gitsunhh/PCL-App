#ifndef DLG_COLOR_PALETTE_H
#define DLG_COLOR_PALETTE_H

#include <QDialog>

namespace Ui {
class dlg_color_palette;
}

class dlg_color_palette : public QDialog
{
    Q_OBJECT

public:
    explicit dlg_color_palette(QWidget *parent = nullptr);
    ~dlg_color_palette();
    int axis();
    int mode();
    void initUi(int axis, int mode);

private:
    Ui::dlg_color_palette *ui;
};

#endif // DLG_COLOR_PALETTE_H
