#ifndef SOLVERPARAMETERSDIALOG_H
#define SOLVERPARAMETERSDIALOG_H

#include <QDialog>

namespace Ui {
  class SolverParametersDialog;
}

class SolverParametersDialog : public QDialog
{
  Q_OBJECT

public:
  explicit SolverParametersDialog(QWidget *parent = 0);
  ~SolverParametersDialog();

  private slots:
  void on_cbbChooseSolver_currentIndexChanged(int index);

private:
  Ui::SolverParametersDialog *ui;
};

#endif // SOLVERPARAMETERSDIALOG_H
