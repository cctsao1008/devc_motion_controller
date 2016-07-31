#include "simulator.h"
#include "ui_simulator.h"

simulator::simulator(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::simulator)
{
    ui->setupUi(this);
}

simulator::~simulator()
{
    delete ui;
}
