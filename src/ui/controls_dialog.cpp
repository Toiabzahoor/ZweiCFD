#include "ZweiCFD/ui/controls_dialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QKeySequenceEdit>
#include <QSettings>

namespace zweicfd {

QVector<KeyAction> ControlsDialog::getDefaultActions() {
    return {
        {"pitch_up", "Pitch Up (Increase Alpha)", "Aerodynamics", QKeySequence(Qt::Key_Up), QKeySequence(Qt::Key_Up)},
        {"pitch_down", "Pitch Down (Decrease Alpha)", "Aerodynamics", QKeySequence(Qt::Key_Down), QKeySequence(Qt::Key_Down)},
        {"camber_up", "Increase Camber", "Aerodynamics", QKeySequence("Shift+C"), QKeySequence("Shift+C")},
        {"camber_down", "Decrease Camber", "Aerodynamics", QKeySequence("C"), QKeySequence("C")},
        {"thickness_up", "Increase Thickness", "Aerodynamics", QKeySequence("Shift+T"), QKeySequence("Shift+T")},
        {"thickness_down", "Decrease Thickness", "Aerodynamics", QKeySequence("T"), QKeySequence("T")},
        {"toggle_flap", "Toggle Flapping", "Aerodynamics", QKeySequence("F"), QKeySequence("F")},
        
        {"speed_up", "Increase Airspeed", "Flow & Air", QKeySequence(Qt::Key_Right), QKeySequence(Qt::Key_Right)},
        {"speed_down", "Decrease Airspeed", "Flow & Air", QKeySequence(Qt::Key_Left), QKeySequence(Qt::Key_Left)},
        {"rake_up", "Move Streamline Rake Up", "Flow & Air", QKeySequence("Shift+Up"), QKeySequence("Shift+Up")},
        {"rake_down", "Move Streamline Rake Down", "Flow & Air", QKeySequence("Shift+Down"), QKeySequence("Shift+Down")},
        {"lines_inc", "Increase Line Count", "Flow & Air", QKeySequence("]"), QKeySequence("]")},
        {"lines_dec", "Decrease Line Count", "Flow & Air", QKeySequence("["), QKeySequence("[")},
        {"line_width_up", "Increase Line Thickness", "Flow & Air", QKeySequence("Shift+]"), QKeySequence("Shift+]")},
        {"line_width_down", "Decrease Line Thickness", "Flow & Air", QKeySequence("Shift+["), QKeySequence("Shift+[")},
        {"turbo_mode", "Turbo Speed", "Flow & Air", QKeySequence("Space"), QKeySequence("Space")},
        {"reset_flow", "Reset Flow Field", "Flow & Air", QKeySequence("R"), QKeySequence("R")},
        
        {"rotate_up", "Orbit Camera Up (3D)", "Camera & View", QKeySequence("W"), QKeySequence("W")},
        {"rotate_down", "Orbit Camera Down (3D)", "Camera & View", QKeySequence("S"), QKeySequence("S")},
        {"rotate_left", "Orbit Camera Left (3D)", "Camera & View", QKeySequence("A"), QKeySequence("A")},
        {"rotate_right", "Orbit Camera Right (3D)", "Camera & View", QKeySequence("D"), QKeySequence("D")},
        {"pan_up", "Pan Camera Up", "Camera & View", QKeySequence("Shift+W"), QKeySequence("Shift+W")},
        {"pan_down", "Pan Camera Down", "Camera & View", QKeySequence("Shift+S"), QKeySequence("Shift+S")},
        {"pan_left", "Pan Camera Left", "Camera & View", QKeySequence("Shift+A"), QKeySequence("Shift+A")},
        {"pan_right", "Pan Camera Right", "Camera & View", QKeySequence("Shift+D"), QKeySequence("Shift+D")},
        {"zoom_in", "Zoom In", "Camera & View", QKeySequence("PgUp"), QKeySequence("PgUp")},
        {"zoom_out", "Zoom Out", "Camera & View", QKeySequence("PgDown"), QKeySequence("PgDown")},
        {"reset_camera", "Reset Camera View", "Camera & View", QKeySequence("Home"), QKeySequence("Home")},
        
        {"draw_mode", "Toggle Draw Mode", "Tools & Display", QKeySequence("M"), QKeySequence("M")},
        {"clear_draw", "Clear Drawing", "Tools & Display", QKeySequence("Delete"), QKeySequence("Delete")},
        {"cycle_theme", "Cycle Colormap Theme", "Tools & Display", QKeySequence("Tab"), QKeySequence("Tab")},
        {"cycle_display", "Cycle Display Mode", "Tools & Display", QKeySequence("H"), QKeySequence("H")},
        {"toggle_particles", "Toggle Particles", "Tools & Display", QKeySequence("P"), QKeySequence("P")}
    };
}

QMap<QString, QKeySequence> ControlsDialog::loadBindings() {
    QMap<QString, QKeySequence> result;
    QSettings settings("ZweiCFD", "Controls");
    auto defaults = getDefaultActions();
    for (const auto& action : defaults) {
        QString val = settings.value("KeyBindings/" + action.id, action.defaultKey.toString()).toString();
        result[action.id] = QKeySequence(val);
    }
    return result;
}

void ControlsDialog::saveBindings(const QMap<QString, QKeySequence>& bindings) {
    QSettings settings("ZweiCFD", "Controls");
    for (auto it = bindings.begin(); it != bindings.end(); ++it) {
        settings.setValue("KeyBindings/" + it.key(), it.value().toString());
    }
}

ControlsDialog::ControlsDialog(QWidget *parent) : QDialog(parent) {
    setWindowTitle("Controls & Keybindings Menu");
    resize(660, 560);
    setModal(true);

    actions = getDefaultActions();
    auto saved = loadBindings();
    for (auto& action : actions) {
        if (saved.contains(action.id)) {
            action.currentKey = saved[action.id];
        }
    }

    setupUi();
}

void ControlsDialog::setupUi() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QHBoxLayout* topLayout = new QHBoxLayout();
    topLayout->addWidget(new QLabel("Preset Profile:"));
    presetComboBox = new QComboBox(this);
    presetComboBox->addItems({"Custom / Current", "Default (WASD 3D Orbit & Arrows)", "Flight Sim (WASD Pitch/Roll)", "CAD / Panning"});
    topLayout->addWidget(presetComboBox, 1);
    mainLayout->addLayout(topLayout);

    tableWidget = new QTableWidget(this);
    tableWidget->setColumnCount(3);
    tableWidget->setHorizontalHeaderLabels({"Category", "Action", "Assigned Key"});
    tableWidget->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    tableWidget->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    tableWidget->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    tableWidget->verticalHeader()->setVisible(false);
    tableWidget->setSelectionMode(QAbstractItemView::NoSelection);
    tableWidget->setAlternatingRowColors(true);
    mainLayout->addWidget(tableWidget);

    populateTable();

    QHBoxLayout* btnLayout = new QHBoxLayout();
    resetButton = new QPushButton("Reset to Defaults", this);
    btnLayout->addWidget(resetButton);
    btnLayout->addStretch();

    cancelButton = new QPushButton("Cancel", this);
    saveButton = new QPushButton("Apply & Save", this);
    saveButton->setDefault(true);
    saveButton->setStyleSheet("font-weight: bold; background-color: #0275d8; color: white; padding: 5px 15px; border-radius: 4px;");

    btnLayout->addWidget(cancelButton);
    btnLayout->addWidget(saveButton);
    mainLayout->addLayout(btnLayout);

    connect(presetComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ControlsDialog::onPresetChanged);
    connect(resetButton, &QPushButton::clicked, this, &ControlsDialog::onResetDefaults);
    connect(saveButton, &QPushButton::clicked, this, &ControlsDialog::onApply);
    connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
}

void ControlsDialog::populateTable() {
    tableWidget->setRowCount(actions.size());
    for (int i = 0; i < actions.size(); ++i) {
        const auto& act = actions[i];

        QTableWidgetItem* catItem = new QTableWidgetItem(act.category);
        catItem->setFlags(Qt::ItemIsEnabled);
        tableWidget->setItem(i, 0, catItem);

        QTableWidgetItem* nameItem = new QTableWidgetItem(act.name);
        nameItem->setFlags(Qt::ItemIsEnabled);
        tableWidget->setItem(i, 1, nameItem);

        QKeySequenceEdit* keyEdit = new QKeySequenceEdit(act.currentKey, this);
        keyEdit->setMaximumSequenceLength(1);
        connect(keyEdit, &QKeySequenceEdit::keySequenceChanged, this, [this, i](const QKeySequence& seq) {
            actions[i].currentKey = seq;
            presetComboBox->blockSignals(true);
            presetComboBox->setCurrentIndex(0);
            presetComboBox->blockSignals(false);
        });
        tableWidget->setCellWidget(i, 2, keyEdit);
    }
}

void ControlsDialog::applyPreset(int presetIndex) {
    if (presetIndex == 1) {
        actions = getDefaultActions();
    } else if (presetIndex == 2) {
        actions = getDefaultActions();
        for (auto& a : actions) {
            if (a.id == "pitch_up") a.currentKey = QKeySequence("W");
            else if (a.id == "pitch_down") a.currentKey = QKeySequence("S");
            else if (a.id == "speed_up") a.currentKey = QKeySequence("D");
            else if (a.id == "speed_down") a.currentKey = QKeySequence("A");
            else if (a.id == "rotate_up") a.currentKey = QKeySequence("Up");
            else if (a.id == "rotate_down") a.currentKey = QKeySequence("Down");
            else if (a.id == "rotate_left") a.currentKey = QKeySequence("Left");
            else if (a.id == "rotate_right") a.currentKey = QKeySequence("Right");
        }
    } else if (presetIndex == 3) {
        actions = getDefaultActions();
        for (auto& a : actions) {
            if (a.id == "pan_up") a.currentKey = QKeySequence("Up");
            else if (a.id == "pan_down") a.currentKey = QKeySequence("Down");
            else if (a.id == "pan_left") a.currentKey = QKeySequence("Left");
            else if (a.id == "pan_right") a.currentKey = QKeySequence("Right");
            else if (a.id == "pitch_up") a.currentKey = QKeySequence("Shift+Up");
            else if (a.id == "pitch_down") a.currentKey = QKeySequence("Shift+Down");
        }
    }
    populateTable();
}

void ControlsDialog::onPresetChanged(int index) {
    if (index > 0) {
        applyPreset(index);
    }
}

void ControlsDialog::onResetDefaults() {
    presetComboBox->setCurrentIndex(1);
    applyPreset(1);
}

QMap<QString, QKeySequence> ControlsDialog::getBindings() const {
    QMap<QString, QKeySequence> result;
    for (const auto& a : actions) {
        result[a.id] = a.currentKey;
    }
    return result;
}

void ControlsDialog::onApply() {
    auto bindings = getBindings();
    saveBindings(bindings);
    emit bindingsChanged(bindings);
    accept();
}

}
