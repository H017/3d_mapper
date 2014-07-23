#include "transformmatrixdialog.h"
#include "ui_transformmatrixdialog.h"

TransformMatrixDialog::TransformMatrixDialog(QWidget *parent, CloudMerger* cm) :
    QDialog(parent),
    ui(new Ui::TransformMatrixDialog)
{
    ui->setupUi(this);
    this->cm = cm;

    connect(ui->buttonBox,SIGNAL(accepted()),this, SLOT(setTransformMatrix()));
}

TransformMatrixDialog::~TransformMatrixDialog()
{
    delete ui;
}

void TransformMatrixDialog::setTransformMatrix()
{
    float x, y, z, rX, rY, rZ;

    x = ui->xTransLineEdit->text().toFloat();
    y = ui->yTransLineEdit->text().toFloat();
    z = ui->zTransLineEdit->text().toFloat();

    rX = DEG2RAD(ui->xRotLineEdit->text().toFloat());
    rY = DEG2RAD(ui->yRotLineEdit->text().toFloat());
    rZ = DEG2RAD(ui->zRotLineEdit->text().toFloat());

    cm->setTransformMatrix(x, y, z, rX, rY, rZ);

}

