#pragma once

#include <QDialog>
#include <QString>
#include <QKeySequence>
#include <QVector>
#include <QMap>
#include <QTableWidget>
#include <QComboBox>
#include <QPushButton>

namespace zweicfd {

struct KeyAction {
    QString id;
    QString name;
    QString category;
    QKeySequence defaultKey;
    QKeySequence currentKey;
};

class ControlsDialog : public QDialog {
    Q_OBJECT

public:
    explicit ControlsDialog(QWidget *parent = nullptr);
    ~ControlsDialog() override = default;

    static QVector<KeyAction> getDefaultActions();
    static QMap<QString, QKeySequence> loadBindings();
    static void saveBindings(const QMap<QString, QKeySequence>& bindings);

    QMap<QString, QKeySequence> getBindings() const;

signals:
    void bindingsChanged(const QMap<QString, QKeySequence>& bindings);

private slots:
    void onResetDefaults();
    void onApply();
    void onPresetChanged(int index);

private:
    void setupUi();
    void populateTable();
    void applyPreset(int presetIndex);

    QTableWidget* tableWidget;
    QComboBox* presetComboBox;
    QPushButton* resetButton;
    QPushButton* saveButton;
    QPushButton* cancelButton;
    QVector<KeyAction> actions;
};

}
