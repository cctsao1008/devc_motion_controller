#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <QMainWindow>

namespace Ui {
class simulator;
}

class simulator : public QMainWindow
{
    Q_OBJECT

public:
    explicit simulator(QWidget *parent = 0);
    ~simulator();

private:
    Ui::simulator *ui;
};

#endif // SIMULATOR_H
