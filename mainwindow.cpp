#include "mainwindow.h"
#include "ui_mainwindow.h"



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    v = new Viewer();
    tmd = new TransformMatrixDialog(parent,v->getCloudMerger());

    ui->verticalLayout->addWidget(&(v->getWidget()));
    connect(ui->actionSnapshot,SIGNAL(triggered()),v,SLOT(snapshot()));
    connect(ui->actionShow_Dialog,SIGNAL(triggered()),tmd,SLOT(show()));
    connect(ui->actionSave_Map,SIGNAL(triggered()),v,SLOT(saveMap()));
    connect(ui->actionLoad_Map,SIGNAL(triggered()),v,SLOT(loadMap()));

    ui->progressBar->setMinimum(0);
    ui->progressBar->setMaximum(v->getMediasCount());

    connect(v,SIGNAL(mergingProgression(int)),ui->progressBar,SLOT(setValue(int)));

}

MainWindow::~MainWindow()
{
    delete tmd;
    delete v;
    delete ui;
}
