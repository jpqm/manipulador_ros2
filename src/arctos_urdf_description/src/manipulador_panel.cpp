#include <array>
#include <memory>
#include <string>
#include <vector>

#include <QDockWidget>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMetaObject>
#include <QPainter>
#include <QPushButton>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace arctos_rviz
{

// --- 1. PAINEL SUPERIOR DIREITO: CÂMERA / YOLO (AO LADO DA ÁREA 3D) ---
class CameraPanel : public rviz_common::Panel
{
public:
  explicit CameraPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent)
  {
    setMinimumSize(360, 270);
  }

  void onInitialize() override
  {
    auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!node_abstraction) {
      return;
    }
    node_ = node_abstraction->get_raw_node();

    sub_image_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 2,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
          return;
        }
        QImage img(
          msg->data.data(),
          static_cast<int>(msg->width),
          static_cast<int>(msg->height),
          static_cast<int>(msg->step),
          QImage::Format_BGR888);
        QImage copia = img.copy();
        QMetaObject::invokeMethod(this, [this, copia]() {
          frame_atual_ = copia;
          update();  // Redesenha apenas este widget sem recalcular o layout do QMainWindow
        });
      });

    // Posiciona este painel na metade superior direita (ao lado da área 3D)
    QTimer::singleShot(0, this, [this]() {
      auto * dock = qobject_cast<QDockWidget *>(parentWidget());
      auto * main_win = qobject_cast<QMainWindow *>(window());
      if (dock && main_win) {
        main_win->addDockWidget(Qt::RightDockWidgetArea, dock);
        main_win->resizeDocks({dock}, {560}, Qt::Horizontal);
      }
    });
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter p(this);
    p.fillRect(rect(), QColor(34, 34, 34));
    if (frame_atual_.isNull()) {
      p.setPen(QColor(170, 170, 170));
      p.drawText(rect(), Qt::AlignCenter, "Aguardando câmera...");
      return;
    }
    QSize alvo = frame_atual_.size().scaled(size(), Qt::KeepAspectRatio);
    int x = (width() - alvo.width()) / 2;
    int y = (height() - alvo.height()) / 2;
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);
    p.drawImage(QRect(x, y, alvo.width(), alvo.height()), frame_atual_);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  QImage frame_atual_;
};

// --- 2. PAINEL INFERIOR: CONTROLE DO MANIPULADOR (LARGURA TOTAL EMBAIXO) ---
class ManipuladorPanel : public rviz_common::Panel
{
public:
  explicit ManipuladorPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent)
  {
    auto * main_layout = new QHBoxLayout(this);
    main_layout->setContentsMargins(8, 6, 8, 6);
    main_layout->setSpacing(12);

    // COLUNA 1 (Esquerda): Feedrate + Juntas + Pose Cartesiana
    auto * col_entradas = new QVBoxLayout();
    col_entradas->setSpacing(4);

    auto * speed_layout = new QHBoxLayout();
    auto * lbl_speed = new QLabel("Velocidade GRBL (Feedrate F):");
    entrada_feedrate_ = new QLineEdit("800");
    entrada_feedrate_->setAlignment(Qt::AlignCenter);
    entrada_feedrate_->setFixedWidth(90);
    speed_layout->addWidget(lbl_speed);
    speed_layout->addWidget(entrada_feedrate_);
    speed_layout->addStretch(1);
    col_entradas->addLayout(speed_layout);

    auto * group_juntas = new QGroupBox("Ângulos das Juntas (graus)");
    auto * grid_juntas = new QGridLayout();
    grid_juntas->setContentsMargins(6, 4, 6, 4);
    grid_juntas->setSpacing(4);
    const std::array<const char *, 6> nomes_j = {"J1", "J2", "J3", "J4", "J5", "J6"};
    for (size_t i = 0; i < 6; ++i) {
      auto * lbl = new QLabel(nomes_j[i]);
      lbl->setFixedWidth(22);
      auto * edit = new QLineEdit("0");
      edit->setAlignment(Qt::AlignCenter);
      int row = static_cast<int>(i / 3);
      int col = static_cast<int>((i % 3) * 2);
      grid_juntas->addWidget(lbl, row, col);
      grid_juntas->addWidget(edit, row, col + 1);
      entradas_juntas_[i] = edit;
    }
    auto * btn_planejar_juntas = new QPushButton("Planejar Juntas");
    btn_planejar_juntas->setMinimumHeight(28);
    grid_juntas->addWidget(btn_planejar_juntas, 2, 0, 1, 6);
    group_juntas->setLayout(grid_juntas);
    col_entradas->addWidget(group_juntas);

    auto * group_pose = new QGroupBox("Posição Alvo na Mesa (mm - Bézier)");
    auto * grid_pose = new QGridLayout();
    grid_pose->setContentsMargins(6, 4, 6, 4);
    grid_pose->setSpacing(4);
    const std::array<const char *, 3> nomes_xyz = {"X", "Y", "Z"};
    const std::array<const char *, 3> vals_xyz = {"-305", "210", "100"};
    for (size_t i = 0; i < 3; ++i) {
      auto * lbl = new QLabel(nomes_xyz[i]);
      auto * edit = new QLineEdit(vals_xyz[i]);
      edit->setAlignment(Qt::AlignCenter);
      grid_pose->addWidget(lbl, 0, static_cast<int>(i * 2));
      grid_pose->addWidget(edit, 0, static_cast<int>(i * 2 + 1));
      entradas_xyz_[i] = edit;
    }
    auto * btn_planejar_pose = new QPushButton("Planejar Pose (Curva Bézier)");
    btn_planejar_pose->setMinimumHeight(28);
    grid_pose->addWidget(btn_planejar_pose, 1, 0, 1, 6);
    group_pose->setLayout(grid_pose);
    col_entradas->addWidget(group_pose);

    // COLUNA 2 (Centro): Botões de Execução, Rotinas e Garra
    auto * col_botoes = new QVBoxLayout();
    col_botoes->setSpacing(5);

    auto * btn_executar = new QPushButton("EXECUTAR MOVIMENTO NO ROBÔ");
    btn_executar->setMinimumHeight(38);
    btn_executar->setStyleSheet("background-color: #FF9800; color: black; font-weight: bold; font-size: 13px;");
    col_botoes->addWidget(btn_executar);

    auto * grid_rotinas = new QGridLayout();
    grid_rotinas->setSpacing(5);

    auto * btn_home = new QPushButton("Planejar Ir para Home");
    btn_home->setMinimumHeight(32);

    auto * btn_lapis = new QPushButton("Planejar Tarefa Lápis");
    btn_lapis->setMinimumHeight(32);
    btn_lapis->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;");

    auto * btn_reconhecer = new QPushButton("Reconhecer (YOLO)");
    btn_reconhecer->setMinimumHeight(32);
    btn_reconhecer->setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;");

    auto * btn_manip = new QPushButton("Planejar Manipular");
    btn_manip->setMinimumHeight(32);
    btn_manip->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;");

    auto * btn_abrir = new QPushButton("Abrir Garra");
    btn_abrir->setMinimumHeight(30);
    auto * btn_fechar = new QPushButton("Fechar Garra");
    btn_fechar->setMinimumHeight(30);

    grid_rotinas->addWidget(btn_home, 0, 0);
    grid_rotinas->addWidget(btn_lapis, 0, 1);
    grid_rotinas->addWidget(btn_reconhecer, 1, 0);
    grid_rotinas->addWidget(btn_manip, 1, 1);
    grid_rotinas->addWidget(btn_abrir, 2, 0);
    grid_rotinas->addWidget(btn_fechar, 2, 1);
    col_botoes->addLayout(grid_rotinas);

    auto * btn_calibrar = new QPushButton("Capturar Calibração");
    btn_calibrar->setMinimumHeight(30);
    btn_calibrar->setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;");
    col_botoes->addWidget(btn_calibrar);
    col_botoes->addStretch(1);

    // COLUNA 3 (Direita): Status + Log / Detecção amplo com fonte 22px
    auto * col_log = new QVBoxLayout();
    col_log->setSpacing(4);

    status_lbl_ = new QLabel("Aguardando comando...");
    status_lbl_->setStyleSheet("color: #ddd; font-weight: bold; font-size: 14px;");
    status_lbl_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    status_lbl_->setWordWrap(true);
    col_log->addWidget(status_lbl_);

    auto * log_group = new QGroupBox("Log / Detecção");
    auto * log_layout = new QVBoxLayout(log_group);
    log_layout->setContentsMargins(4, 4, 4, 4);
    log_text_ = new QTextEdit();
    log_text_->setReadOnly(true);
    log_text_->setStyleSheet(
      "background-color: #1e1e1e; color: #0f0; font-family: monospace; font-size: 27px;");
    log_layout->addWidget(log_text_);
    col_log->addWidget(log_group, 1);

    main_layout->addLayout(col_entradas, 3);
    main_layout->addLayout(col_botoes, 3);
    main_layout->addLayout(col_log, 5);

    // Conexões dos botões e do campo de velocidade (Feedrate F)
    QObject::connect(entrada_feedrate_, &QLineEdit::editingFinished, this, [this]() { publicarFeedrate(); });
    QObject::connect(btn_planejar_juntas, &QPushButton::clicked, this, [this]() { publicarJuntas(); });
    QObject::connect(btn_planejar_pose, &QPushButton::clicked, this, [this]() { publicarPose(); });
    QObject::connect(btn_executar, &QPushButton::clicked, this, [this]() { chamarTrigger(cli_executar_); });
    QObject::connect(btn_home, &QPushButton::clicked, this, [this]() { chamarTrigger(cli_home_); });
    QObject::connect(btn_lapis, &QPushButton::clicked, this, [this]() { chamarTrigger(cli_lapis_); });
    QObject::connect(btn_reconhecer, &QPushButton::clicked, this, [this]() { chamarTrigger(cli_reconhecer_); });
    QObject::connect(btn_manip, &QPushButton::clicked, this, [this]() { chamarTrigger(cli_manipular_); });
    QObject::connect(btn_abrir, &QPushButton::clicked, this, [this]() { chamarTrigger(cli_abrir_); });
    QObject::connect(btn_fechar, &QPushButton::clicked, this, [this]() { chamarTrigger(cli_fechar_); });
    QObject::connect(btn_calibrar, &QPushButton::clicked, this, [this]() { chamarTrigger(cli_calibrar_); });
  }

  void onInitialize() override
  {
    auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!node_abstraction) {
      return;
    }
    node_ = node_abstraction->get_raw_node();

    pub_juntas_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/planejar_juntas", 10);
    pub_pose_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/planejar_pose", 10);
    pub_feedrate_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/configurar_feedrate", 10);

    cli_home_ = node_->create_client<std_srvs::srv::Trigger>("/planejar_home");
    cli_lapis_ = node_->create_client<std_srvs::srv::Trigger>("/planejar_rotina_lapis");
    cli_manipular_ = node_->create_client<std_srvs::srv::Trigger>("/planejar_manipular");
    cli_executar_ = node_->create_client<std_srvs::srv::Trigger>("/executar");
    cli_reconhecer_ = node_->create_client<std_srvs::srv::Trigger>("/reconhecer");
    cli_abrir_ = node_->create_client<std_srvs::srv::Trigger>("/abrir_garra");
    cli_fechar_ = node_->create_client<std_srvs::srv::Trigger>("/fechar_garra");
    cli_calibrar_ = node_->create_client<std_srvs::srv::Trigger>("/calibrar");

    sub_status_ = node_->create_subscription<std_msgs::msg::String>(
      "/manipulador_status", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        QString texto = QString::fromStdString(msg->data);
        QMetaObject::invokeMethod(this, [this, texto]() {
          status_lbl_->setText(texto);
          log_text_->append(texto);
        });
      });

    // Posiciona este painel na parte inferior ocupando 100% da largura da janela
    QTimer::singleShot(0, this, [this]() {
      auto * dock = qobject_cast<QDockWidget *>(parentWidget());
      auto * main_win = qobject_cast<QMainWindow *>(window());
      if (dock && main_win) {
        main_win->setCorner(Qt::BottomLeftCorner, Qt::BottomDockWidgetArea);
        main_win->setCorner(Qt::BottomRightCorner, Qt::BottomDockWidgetArea);
        main_win->addDockWidget(Qt::BottomDockWidgetArea, dock);
        main_win->resizeDocks({dock}, {250}, Qt::Vertical);
      }
    });
  }

private:
  double lerFeedrate() const
  {
    bool ok = false;
    double f = entrada_feedrate_->text().toDouble(&ok);
    return (ok && f > 1.0) ? f : 800.0;
  }

  void publicarFeedrate()
  {
    if (!pub_feedrate_) {
      return;
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data.push_back(lerFeedrate());
    pub_feedrate_->publish(msg);
  }

  void publicarJuntas()
  {
    if (!pub_juntas_) {
      return;
    }
    std_msgs::msg::Float64MultiArray msg;
    for (size_t i = 0; i < 6; ++i) {
      bool ok = false;
      double val = entradas_juntas_[i]->text().toDouble(&ok);
      if (!ok) {
        status_lbl_->setText("Erro: ângulos J1..J6 inválidos");
        return;
      }
      msg.data.push_back(val);
    }
    msg.data.push_back(lerFeedrate());
    pub_juntas_->publish(msg);
  }

  void publicarPose()
  {
    if (!pub_pose_) {
      return;
    }
    std_msgs::msg::Float64MultiArray msg;
    for (size_t i = 0; i < 3; ++i) {
      bool ok = false;
      double val = entradas_xyz_[i]->text().toDouble(&ok);
      if (!ok) {
        status_lbl_->setText("Erro: coordenadas X, Y, Z inválidas");
        return;
      }
      msg.data.push_back(val);
    }
    msg.data.push_back(1.0);  // usar_referencial_mesa = True
    msg.data.push_back(lerFeedrate());
    pub_pose_->publish(msg);
  }

  void chamarTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client)
  {
    if (!client) {
      return;
    }
    if (!client->service_is_ready()) {
      status_lbl_->setText("Serviço indisponível (aguarde o manipulador_node iniciar)");
      return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(req);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_juntas_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pose_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_feedrate_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_home_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_lapis_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_manipular_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_executar_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_reconhecer_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_abrir_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_fechar_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_calibrar_;

  QLineEdit * entrada_feedrate_{nullptr};
  std::array<QLineEdit *, 6> entradas_juntas_{};
  std::array<QLineEdit *, 3> entradas_xyz_{};
  QLabel * status_lbl_{nullptr};
  QTextEdit * log_text_{nullptr};
};

}  // namespace arctos_rviz

PLUGINLIB_EXPORT_CLASS(arctos_rviz::CameraPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(arctos_rviz::ManipuladorPanel, rviz_common::Panel)
