#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "viewer.h"
#include "transformmatrixdialog.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    Viewer* v = new Viewer();
    TransformMatrixDialog* tmd = new TransformMatrixDialog(parent,v->getCloudMerger());

    ui->verticalLayout->addWidget(&(v->getWidget()));
    connect(ui->snapShotButton,SIGNAL(clicked()),v,SLOT(snapshot()));
    connect(ui->actionShow_Dialog,SIGNAL(triggered()),tmd,SLOT(show()));

}

MainWindow::~MainWindow()
{
    delete ui;
}
