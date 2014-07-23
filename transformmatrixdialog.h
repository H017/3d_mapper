#ifndef TRANSFORMMATRIXDIALOG_H
#define TRANSFORMMATRIXDIALOG_H

#include <QDialog>
#include "cloudmerger.h"

namespace Ui {
class TransformMatrixDialog;
}

class TransformMatrixDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit TransformMatrixDialog(QWidget *parent = 0,CloudMerger* cm = NULL);
    ~TransformMatrixDialog();

public slots:
    void setTransformMatrix();
    
private:
    Ui::TransformMatrixDialog *ui;
    CloudMerger* cm;

};

#endif // TRANSFORMMATRIXDIALOG_H
