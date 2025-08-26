function PendulumControlGUI()
    %PENDULUMCONTROLGUI_SWINGUP GUI actualizada para swing-up con motor stepper
    %   Versión sincronizada con pendulum-controller-stepper-swingup.ino
    %   Incluye monitoreo del swing-up y transición automática al control
    
    %% Inicialización de variables globales
    global gui_data;
    
    % Estructura para datos de la GUI
    gui_data = struct();
    gui_data.ctrl = []; % Objeto de comunicación
    gui_data.is_connected = false;
    gui_data.is_running = false;
    gui_data.current_controller = 0; % 0=PID, 1=LQR
    
    % Matrices del sistema LQR (fijas del código Arduino)
    gui_data.A_aug = [0,    1,    0,        0,      0;
                      0,    0,    0,        0,      0;
                      0,    0,    0,        1,      0;
                      0,    0,    55.85,    -0.374, 0;
                      1,    0,    0,        0,      0];
                     
    gui_data.B_aug = [0; 1.0; 0; 0.803; 0];
    gui_data.C_aug = eye(5);
    gui_data.D_aug = zeros(5,1);
    
    % Parámetros por defecto (sincronizados con Arduino)
    gui_data.pid_gains = [0.1, 0.0008, 0.012]; % [Kp, Ki, Kd]
    gui_data.q_diag = [1, 1, 1, 1, 1]; % Diagonal de Q
    gui_data.r_weight = 1; % Peso R
    gui_data.lqr_gains = [0, 0, 0, 0, 0]; % K calculado
    
    % Datos para gráficas (6 observaciones según Arduino)
    gui_data.max_points = 1500; % Más puntos para swing-up
    gui_data.time_data = [];
    gui_data.sample_data = []; % Índice de muestra
    gui_data.theta_data = []; % Ángulo péndulo
    gui_data.phi_data = []; % Ángulo rotor (stepper)
    gui_data.dtheta_data = []; % Velocidad péndulo
    gui_data.dphi_data = []; % Velocidad rotor
    gui_data.swing_impulse_data = []; % Ángulo de impulso (obs[4])
    gui_data.energy_error_data = []; % Energía o error integral (obs[5])
    gui_data.status_data = []; % Estado del sistema
    
    % Variables de timing
    gui_data.start_timestamp = [];
    gui_data.last_timestamp = 0;
    gui_data.last_successful_read = tic;
    gui_data.sample_counter = 0;
    
    % Estados del sistema (sincronizados con Arduino)
    gui_data.STATE_IDLE = 0;
    gui_data.STATE_SWING_UP = 1;
    gui_data.STATE_CONTROL = 2;
    gui_data.STATE_STOPPED = 3;
    
    % Status codes (sincronizados con Arduino)
    gui_data.STATUS_OK = 0;
    gui_data.STATUS_STP_MOVING = 1;
    gui_data.STATUS_SWING_UP = 2;
    gui_data.STATUS_CONTROL_ACTIVE = 3;
    gui_data.STATUS_UPRIGHT_ACHIEVED = 4;
    
    % Variables para análisis de swing-up
    gui_data.swing_start_time = [];
    gui_data.upright_achieved = false;
    gui_data.max_theta_achieved = 0;
    gui_data.swing_phase_info = '';
    
    %% Crear interfaz gráfica centrada
    create_centered_gui();
    create_gui_layout();
    initialize_plots();
    
    % Timer para actualización más frecuente durante swing-up
    gui_data.timer = timer('ExecutionMode', 'fixedRate', ...
                          'Period', 0.04, ... % 40ms para capturar swing-up
                          'TimerFcn', @update_display);
    
    fprintf('=== GUI SWING-UP CON STEPPER INICIADA ===\n');
    fprintf('Sincronizada con pendulum-controller-stepper-swingup.ino\n');
    fprintf('Funcionalidades:\n');
    fprintf('• Monitoreo del swing-up en tiempo real\n');
    fprintf('• Análisis de energía del péndulo\n');
    fprintf('• Transición automática al control\n');
    fprintf('• Control PID/LQR con motor stepper\n');
    fprintf('Por favor conecte al puerto serial.\n');
end

function create_centered_gui()
    global gui_data;
    
    % Obtener dimensiones de pantalla
    screen_size = get(0, 'ScreenSize');
    screen_width = screen_size(3);
    screen_height = screen_size(4);
    
    % Dimensiones optimizadas para swing-up
    fig_width = 1200;
    fig_height = 790;
    
    % Calcular posición centrada
    fig_x = (screen_width - fig_width) / 2;
    fig_y = (screen_height - fig_height) / 2;
    
    % Crear figura principal
    gui_data.fig = figure('Name', 'Control Péndulo Invertido', ...
                         'Position', [fig_x, fig_y, fig_width, fig_height], ...
                         'Resize', 'on', ...
                         'CloseRequestFcn', @close_gui, ...
                         'MenuBar', 'none', ...
                         'ToolBar', 'none');
    
    figure(gui_data.fig);
end

function create_gui_layout()
    global gui_data;
    gui_data.y_offset = 100;
    %% Panel de parámetros (izquierda)
    gui_data.param_panel = uipanel('Parent', gui_data.fig, ...
                                  'Title', 'Parámetros y Control', ...
                                  'Position', [0.02, 0.02, 0.32, 0.96], ...
                                  'FontSize', 10, ...
                                  'FontWeight', 'bold');
    
    create_connection_controls();
    create_controller_selection();
    create_pid_controls();
    create_lqr_controls();
    create_swing_up_info();
    create_control_buttons();
    
    %% Panel de gráficas (derecha)
    gui_data.plot_panel = uipanel('Parent', gui_data.fig, ...
                                 'Title', 'Monitoreo Control', ...
                                 'Position', [0.36, 0.02, 0.62, 0.96], ...
                                 'FontSize', 10, ...
                                 'FontWeight', 'bold');
end



function create_connection_controls()
    global gui_data;
    
    % Conexión serial
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', 'Puerto Serial:', ...
             'Position', [10, 800-gui_data.y_offset, 100, 20], ...
             'HorizontalAlignment', 'left', ...
             'FontSize', 10);
         
    gui_data.port_edit = uicontrol('Parent', gui_data.param_panel, ...
                                  'Style', 'edit', ...
                                  'String', 'COM10', ...
                                  'Position', [120, 800-gui_data.y_offset, 80, 25], ...
                                  'FontSize', 9);
                              
    gui_data.connect_btn = uicontrol('Parent', gui_data.param_panel, ...
                                    'Style', 'pushbutton', ...
                                    'String', 'Conectar', ...
                                    'Position', [210, 800-gui_data.y_offset, 80, 25], ...
                                    'Callback', @connect_callback, ...
                                    'FontSize', 9, ...
                                    'BackgroundColor', [0, 0.8, 0]);
    
    gui_data.refresh_btn = uicontrol('Parent', gui_data.param_panel, ...
                                    'Style', 'pushbutton', ...
                                    'String', 'Actualizar', ...
                                    'Position', [300, 800-gui_data.y_offset, 70, 25], ...
                                    'Callback', @refresh_ports_callback, ...
                                    'FontSize', 9, ...
                                    'BackgroundColor', [0.9, 0.9, 0.9]);
end

function create_controller_selection()
    global gui_data;
    
    % Selección de controlador
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '═══ SELECCIÓN DE CONTROLADOR ═══', ...
             'Position', [10, 760-gui_data.y_offset, 360, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    % Botones de selección
    gui_data.pid_btn = uicontrol('Parent', gui_data.param_panel, ...
                                'Style', 'pushbutton', ...
                                'String', 'PID', ...
                                'Position', [45, 720-gui_data.y_offset, 130, 35], ...
                                'FontSize', 12, ...
                                'FontWeight', 'bold', ...
                                'BackgroundColor', [0.2, 0.8, 0.2], ...
                                'ForegroundColor', 'white', ...
                                'Callback', @(~,~) select_controller(0));
                            
    gui_data.lqr_btn = uicontrol('Parent', gui_data.param_panel, ...
                                'Style', 'pushbutton', ...
                                'String', 'LQR', ...
                                'Position', [205, 720-gui_data.y_offset, 130, 35], ...
                                'FontSize', 12, ...
                                'FontWeight', 'bold', ...
                                'BackgroundColor', [0.7, 0.7, 0.7], ...
                                'ForegroundColor', 'black', ...
                                'Callback', @(~,~) select_controller(1));
end

function create_pid_controls()
    global gui_data;
    
    % Parámetros PID
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '── PARÁMETROS PID ──', ...
             'Position', [10, 680-gui_data.y_offset, 200, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    labels = {'Kp:', 'Ki:', 'Kd:'};
    defaults = {'0.1', '0.0008', '0.012'}; % Valores del Arduino
    
    gui_data.pid_edits = [];
    
    for i = 1:3
        y_pos = 650 - (i-1)*30;
        
        uicontrol('Parent', gui_data.param_panel, ...
                 'Style', 'text', ...
                 'String', labels{i}, ...
                 'Position', [20, y_pos-gui_data.y_offset, 30, 20], ...
                 'HorizontalAlignment', 'left', ...
                 'FontSize', 9);
             
        gui_data.pid_edits(i) = uicontrol('Parent', gui_data.param_panel, ...
                                         'Style', 'edit', ...
                                         'String', defaults{i}, ...
                                         'Position', [60, y_pos-gui_data.y_offset, 100, 25], ...
                                         'FontSize', 9, ...
                                         'BackgroundColor', 'white');
    end
end

function create_lqr_controls()
    global gui_data;
    
    % Parámetros LQR
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '── PARÁMETROS LQR ──', ...
             'Position', [10, 560-gui_data.y_offset, 200, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    % Pesos Q
    q_labels = {'Q1:', 'Q2:', 'Q3:', 'Q4:', 'Q5:'};
    q_defaults = {'1', '1', '1', '1', '1'};
    
    gui_data.q_edits = [];
    
    % Primera fila: Q1, Q2, Q3
    for i = 1:3
        x_pos = 20 + (i-1)*85;
        
        uicontrol('Parent', gui_data.param_panel, ...
                 'Style', 'text', ...
                 'String', q_labels{i}, ...
                 'Position', [x_pos, 530-gui_data.y_offset, 25, 20], ...
                 'HorizontalAlignment', 'left', ...
                 'FontSize', 8);
             
        gui_data.q_edits(i) = uicontrol('Parent', gui_data.param_panel, ...
                                       'Style', 'edit', ...
                                       'String', q_defaults{i}, ...
                                       'Position', [x_pos+25, 530-gui_data.y_offset, 50, 25], ...
                                       'FontSize', 8);
    end
    
    % Segunda fila: Q4, Q5
    for i = 4:5
        x_pos = 20 + (i-4)*85;
        
        uicontrol('Parent', gui_data.param_panel, ...
                 'Style', 'text', ...
                 'String', q_labels{i}, ...
                 'Position', [x_pos, 500-gui_data.y_offset, 25, 20], ...
                 'HorizontalAlignment', 'left', ...
                 'FontSize', 8);
             
        gui_data.q_edits(i) = uicontrol('Parent', gui_data.param_panel, ...
                                       'Style', 'edit', ...
                                       'String', q_defaults{i}, ...
                                       'Position', [x_pos+25, 500-gui_data.y_offset, 50, 25], ...
                                       'FontSize', 8);
    end
    
    % Peso R y botón calcular
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', 'R:', ...
             'Position', [190, 500-gui_data.y_offset, 20, 20], ...
             'FontSize', 8);
         
    gui_data.r_edit = uicontrol('Parent', gui_data.param_panel, ...
                               'Style', 'edit', ...
                               'String', '1', ...
                               'Position', [215, 500-gui_data.y_offset, 50, 25], ...
                               'FontSize', 8);
    
    gui_data.calc_k_btn = uicontrol('Parent', gui_data.param_panel, ...
                                   'Style', 'pushbutton', ...
                                   'String', 'Calcular K', ...
                                   'Position', [280, 500-gui_data.y_offset, 80, 25], ...
                                   'Callback', @calculate_lqr_gains, ...
                                   'FontSize', 9, ...
                                   'BackgroundColor', [0.9, 0.9, 1]);
    
    % Mostrar ganancias K
    gui_data.k_display = uicontrol('Parent', gui_data.param_panel, ...
                                  'Style', 'text', ...
                                  'String', 'K = [0.000, 0.000, 0.000, 0.000, 0.000]', ...
                                  'Position', [20, 460-gui_data.y_offset, 340, 30], ...
                                  'HorizontalAlignment', 'center', ...
                                  'FontSize', 10);
end

function create_swing_up_info()
    global gui_data;
    
    % Panel de información de swing-up
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '═══ INFORMACIÓN ═══', ...
             'Position', [10, 420-gui_data.y_offset, 360, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    % Estado actual
    gui_data.swing_status = uicontrol('Parent', gui_data.param_panel, ...
                                     'Style', 'text', ...
                                     'String', 'Estado: Sistema Inactivo', ...
                                     'Position', [20, 390-gui_data.y_offset, 340, 20], ...
                                     'HorizontalAlignment', 'center', ...
                                     'FontSize', 11, ...
                                     'FontWeight', 'bold', ...
                                     'BackgroundColor', [0.95, 0.95, 0.95]);
    
    % Tiempo transcurrido
    gui_data.time_display = uicontrol('Parent', gui_data.param_panel, ...
                                     'Style', 'text', ...
                                     'String', 'Tiempo: 0.0 s', ...
                                     'Position', [20, 365-gui_data.y_offset, 160, 20], ...
                                     'FontSize', 8);
    
    % Ángulo máximo alcanzado
    gui_data.max_angle_display = uicontrol('Parent', gui_data.param_panel, ...
                                          'Style', 'text', ...
                                          'String', 'Máx θ: 0.0°', ...
                                          'Position', [200, 365-gui_data.y_offset, 160, 20], ...
                                          'FontSize', 8);
    
    % Energía del péndulo
    gui_data.energy_display = uicontrol('Parent', gui_data.param_panel, ...
                                       'Style', 'text', ...
                                       'String', 'Energía: 0.0', ...
                                       'Position', [20, 340-gui_data.y_offset, 160, 20], ...
                                       'FontSize', 8);
    
    % Fase de swing-up
    gui_data.phase_display = uicontrol('Parent', gui_data.param_panel, ...
                                      'Style', 'text', ...
                                      'String', 'Fase: ---', ...
                                      'Position', [200, 340-gui_data.y_offset, 160, 20], ...
                                      'FontSize', 8);
end

function create_control_buttons()
    global gui_data;
    
    % Estado de conexión
    gui_data.status_text = uicontrol('Parent', gui_data.param_panel, ...
                                    'Style', 'text', ...
                                    'String', 'Estado: Desconectado', ...
                                    'Position', [20, 300-gui_data.y_offset, 340, 25], ...
                                    'HorizontalAlignment', 'center', ...
                                    'FontSize', 11, ...
                                    'FontWeight', 'bold', ...
                                    'ForegroundColor', [0.8, 0, 0], ...
                                    'BackgroundColor', [0.95, 0.95, 0.95]);
    
    % Botones principales
    gui_data.start_btn = uicontrol('Parent', gui_data.param_panel, ...
                                  'Style', 'pushbutton', ...
                                  'String', '▶ INICIAR', ...
                                  'Position', [20, 250-gui_data.y_offset, 160, 40], ...
                                  'Callback', @start_control, ...
                                  'FontSize', 11, ...
                                  'FontWeight', 'bold', ...
                                  'BackgroundColor', [0, 0.8, 0], ...
                                  'ForegroundColor', 'white', ...
                                  'Enable', 'off');
    
    gui_data.stop_btn = uicontrol('Parent', gui_data.param_panel, ...
                                 'Style', 'pushbutton', ...
                                 'String', '⏹ PARAR', ...
                                 'Position', [200, 250-gui_data.y_offset, 160, 40], ...
                                 'Callback', @stop_control, ...
                                 'FontSize', 11, ...
                                 'FontWeight', 'bold', ...
                                 'BackgroundColor', [0.8, 0, 0], ...
                                 'ForegroundColor', 'white', ...
                                 'Enable', 'off');
    
    % Botones secundarios
    button_width = 80;
    spacing = 10;
    
    gui_data.home_btn = uicontrol('Parent', gui_data.param_panel, ...
                                 'Style', 'pushbutton', ...
                                 'String', 'Home', ...
                                 'Position', [20, 200-gui_data.y_offset, button_width, 30], ...
                                 'Callback', @set_home, ...
                                 'FontSize', 9, ...
                                 'Enable', 'off');
    
    gui_data.save_btn = uicontrol('Parent', gui_data.param_panel, ...
                                 'Style', 'pushbutton', ...
                                 'String', 'Guardar', ...
                                 'Position', [20 + button_width + spacing, 200-gui_data.y_offset, button_width, 30], ...
                                 'Callback', @save_data, ...
                                 'FontSize', 9, ...
                                 'BackgroundColor', [0, 0, 0.8], ...
                                 'ForegroundColor', 'white', ...
                                 'Enable', 'off');    
end

function initialize_plots()
    global gui_data;
    
    % Crear subplots 3x2 optimizados para swing-up con mejor espaciado
    
    % 1. Estado del sistema
    gui_data.ax1 = subplot(3, 2, 1, 'Parent', gui_data.plot_panel);
    gui_data.line1 = plot(gui_data.ax1, NaN, NaN, 'k');
    title(gui_data.ax1, 'Estado del Sistema', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax1, 'Muestra');
    ylabel(gui_data.ax1, 'Estado');
    grid(gui_data.ax1, 'on');
    ylim(gui_data.ax1, [-0.5, 4.5]);
    set(gui_data.ax1, 'YTick', 0:4, 'YTickLabel', {'Idle', 'Swing', 'Control', 'Stop', 'Upright'});
    xlim(gui_data.ax1, [0, 50]); % Límite inicial
    
    % 2. Ángulo del péndulo con setpoint - MEJORADO
    gui_data.ax2 = subplot(3, 2, 2, 'Parent', gui_data.plot_panel);
    hold(gui_data.ax2, 'on');
    gui_data.line2a = plot(gui_data.ax2, NaN, NaN, 'r-', 'LineWidth', 2, 'DisplayName', 'Setpoint (180°)');
    gui_data.line2b = plot(gui_data.ax2, NaN, NaN, 'k', 'DisplayName', 'Ángulo Péndulo');
    title(gui_data.ax2, 'Ángulo del Péndulo', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax2, 'Muestra');
    ylabel(gui_data.ax2, 'Grados');
    legend(gui_data.ax2, 'show', 'Location', 'best', 'FontSize', 8);
    grid(gui_data.ax2, 'on');
    ylim(gui_data.ax2, [0, 360]);
    xlim(gui_data.ax2, [0, 50]);
    hold(gui_data.ax2, 'off');
    
    % 3. Ángulo del rotor (stepper) - MEJORADO
    gui_data.ax3 = subplot(3, 2, 3, 'Parent', gui_data.plot_panel);
    gui_data.line3 = plot(gui_data.ax3, NaN, NaN, 'k');
    title(gui_data.ax3, 'Ángulo Rotor (Stepper)', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax3, 'Muestra');
    ylabel(gui_data.ax3, 'Grados');
    grid(gui_data.ax3, 'on');
    xlim(gui_data.ax3, [0, 50]);
    
    % 4. Velocidades angulares - MEJORADO
    gui_data.ax4 = subplot(3, 2, 4, 'Parent', gui_data.plot_panel);
    hold(gui_data.ax4, 'on');
    gui_data.line4a = plot(gui_data.ax4, NaN, NaN,  'k', 'DisplayName', 'ω péndulo');
    gui_data.line4b = plot(gui_data.ax4, NaN, NaN, 'r-', 'DisplayName', 'ω rotor');
    title(gui_data.ax4, 'Velocidades Angulares', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax4, 'Muestra');
    ylabel(gui_data.ax4, 'Grados/s');
    legend(gui_data.ax4, 'show', 'Location', 'best', 'FontSize', 8);
    grid(gui_data.ax4, 'on');
    xlim(gui_data.ax4, [0, 50]);
    hold(gui_data.ax4, 'off');
    
    % 5. Impulso de swing-up - MEJORADO CON MEJOR VISUALIZACIÓN
    gui_data.ax5 = subplot(3, 2, 5, 'Parent', gui_data.plot_panel);
    gui_data.line5 = plot(gui_data.ax5, NaN, NaN, 'k');
    title(gui_data.ax5, 'Impulso Swing-up', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax5, 'Muestra');
    ylabel(gui_data.ax5, 'Ángulo Impulso (°)');
    grid(gui_data.ax5, 'on');
    xlim(gui_data.ax5, [0, 50]);
    ylim(gui_data.ax5, [-80, 80]); % Rango inicial más amplio
    
    % 6. Energía/Error - MEJORADO
    gui_data.ax6 = subplot(3, 2, 6, 'Parent', gui_data.plot_panel);
    gui_data.line6 = plot(gui_data.ax6, NaN, NaN, 'k');
    title(gui_data.ax6, 'Energía/Error Integral', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax6, 'Muestra');
    ylabel(gui_data.ax6, 'Valor');
    grid(gui_data.ax6, 'on');
    xlim(gui_data.ax6, [0, 50]);
    
    % CONFIGURAR TODAS LAS GRÁFICAS PARA MEJOR RENDIMIENTO
    all_axes = [gui_data.ax1, gui_data.ax2, gui_data.ax3, gui_data.ax4, gui_data.ax5, gui_data.ax6];
    
    for i = 1:length(all_axes)
        % Mejorar renderizado
        set(all_axes(i), 'FontSize', 9);
        set(all_axes(i), 'GridAlpha', 0.3);
        set(all_axes(i), 'MinorGridAlpha', 0.1);
        % Asegurar que los ejes se actualicen correctamente
        set(all_axes(i), 'XLimMode', 'auto');
        set(all_axes(i), 'YLimMode', 'auto');
    end
    
    % Ajustar espaciado entre subplots
    set(gui_data.plot_panel, 'TitlePosition', 'centertop');
    
    fprintf('Gráficas inicializadas con configuración optimizada\n');
end

% FUNCIÓN para verificar y corregir datos antes de graficar
function [x_data, y_data] = validate_plot_data(x_raw, y_raw)
    % Validar que los datos sean apropiados para graficar
    
    if isempty(x_raw) || isempty(y_raw)
        x_data = NaN;
        y_data = NaN;
        return;
    end
    
    % Asegurar que sean vectores fila
    x_raw = x_raw(:)';
    y_raw = y_raw(:)';
    
    % Verificar que tengan la misma longitud
    if length(x_raw) ~= length(y_raw)
        min_len = min(length(x_raw), length(y_raw));
        x_raw = x_raw(1:min_len);
        y_raw = y_raw(1:min_len);
    end
    
    % Remover valores NaN o Inf
    valid_idx = isfinite(x_raw) & isfinite(y_raw);
    
    if ~any(valid_idx)
        x_data = NaN;
        y_data = NaN;
        return;
    end
    
    x_data = x_raw(valid_idx);
    y_data = y_raw(valid_idx);
    
    % Asegurar que tengamos al menos 2 puntos para graficar
    if length(x_data) < 2
        x_data = [x_data, x_data(end)+1];
        y_data = [y_data, y_data(end)];
    end
end

%% Callbacks principales

function select_controller(controller_type)
    global gui_data;
    
    gui_data.current_controller = controller_type;
    
    % Actualizar apariencia
    if controller_type == 0
        set(gui_data.pid_btn, 'BackgroundColor', [0.2, 0.8, 0.2], 'ForegroundColor', 'white');
        set(gui_data.lqr_btn, 'BackgroundColor', [0.7, 0.7, 0.7], 'ForegroundColor', 'black');
        fprintf('Controlador PID seleccionado\n');
    else
        set(gui_data.pid_btn, 'BackgroundColor', [0.7, 0.7, 0.7], 'ForegroundColor', 'black');
        set(gui_data.lqr_btn, 'BackgroundColor', [0.2, 0.8, 0.2], 'ForegroundColor', 'white');
        fprintf('Controlador LQR seleccionado\n');
    end
    
    % Enviar al Arduino
    if gui_data.is_connected
        try
            controller_action = [controller_type, 0, 0, 0, 0, 0];
            gui_data.ctrl.step(gui_data.ctrl.CMD_SELECT_CONTROLLER, controller_action);
        catch ME
            fprintf('Error enviando selección: %s\n', ME.message);
        end
    end
end

function connect_callback(~, ~)
    global gui_data;
    
    if ~gui_data.is_connected
        % Intentar conectar
        port = get(gui_data.port_edit, 'String');
        
        if isempty(gui_data.ctrl)
            gui_data.ctrl = ControlComms(2.0, 0); % timeout=2s, debug=none
        end
        
        status = gui_data.ctrl.connect(port, 500000); % Baudrate del Arduino
        
        if status == gui_data.ctrl.STATUS_OK
            % Dar tiempo al Arduino para inicializar
            pause(2);
            
            gui_data.is_connected = true;
            set(gui_data.connect_btn, 'String', 'Desconectar', 'BackgroundColor', [0.8, 0, 0]);
            set(gui_data.status_text, 'String', 'Estado: Conectado - Sistema Listo', 'ForegroundColor', [0, 0.6, 0]);
            set([gui_data.start_btn, gui_data.stop_btn, gui_data.home_btn, gui_data.save_btn], 'Enable', 'on'); 
            
            % Configuración inicial sincronizada con Arduino
            try
                base_action = [0, 0, 0, 0, 0, 0];
                
                % Comando SET_STEP_MODE (modo 1/16)
                step_action = base_action;
                step_action(1) = 4; % Modo 1/16 (div_per_step = 16)
                gui_data.ctrl.step(gui_data.ctrl.CMD_SET_STEP_MODE, step_action);
                pause(0.2);
                
                % Comando SET_HOME
                gui_data.ctrl.step(gui_data.ctrl.CMD_SET_HOME, base_action);
                pause(0.2);
                
                % Enviar selección de controlador
                controller_action = base_action;
                controller_action(1) = gui_data.current_controller;
                gui_data.ctrl.step(gui_data.ctrl.CMD_SELECT_CONTROLLER, controller_action);
                pause(0.2);
                
                fprintf('Arduino configurado: Home establecido\n');
                
            catch ME
                fprintf('Advertencia configurando Arduino: %s\n', ME.message);
            end
            
            fprintf('=== CONECTADO EXITOSAMENTE ===\n');
            fprintf('Puerto: %s | Baudrate: 500000\n', port);
            fprintf('Sistema listo para swing-up\n');
            
        else
            msgbox('Error al conectar con Arduino. Verifique el puerto y que esté ejecutando el código correcto.', 'Error', 'error');
        end
    else
        % Desconectar
        disconnect_arduino();
    end
end

function refresh_ports_callback(~, ~)
    global gui_data;
    
    try
        temp_ctrl = ControlComms(1.0, 0);
        available_ports = temp_ctrl.getSerialList();
        
        if ~isempty(available_ports)
            fprintf('Puertos disponibles: %s\n', strjoin(available_ports, ', '));
            current_port = get(gui_data.port_edit, 'String');
            if isempty(current_port) || ~any(strcmp(available_ports, current_port))
                set(gui_data.port_edit, 'String', available_ports{1});
            end
        else
            fprintf('No se encontraron puertos seriales\n');
        end
    catch ME
        fprintf('Error obteniendo puertos: %s\n', ME.message);
    end
end

function calculate_lqr_gains(~, ~)
    global gui_data;
    
    try
        % Leer valores de la interfaz
        Q_diag = zeros(1, 5);
        for i = 1:5
            Q_diag(i) = str2double(get(gui_data.q_edits(i), 'String'));
        end
        R_val = str2double(get(gui_data.r_edit, 'String'));
        
        % Validar valores
        if any(isnan(Q_diag)) || isnan(R_val) || any(Q_diag <= 0) || R_val <= 0
            msgbox('Todos los valores deben ser números positivos', 'Error', 'error');
            return;
        end
        
        % Crear matrices
        Q = diag(Q_diag);
        R = R_val;
        
        % Calcular ganancias LQR
        gui_data.lqr_gains = lqr(gui_data.A_aug, gui_data.B_aug, Q, R);
        
        % Mostrar resultado
        k_str = sprintf('K = [%.3f, %.3f, %.3f, %.3f, %.3f]', gui_data.lqr_gains);
        set(gui_data.k_display, 'String', k_str);
        
        % Enviar al Arduino
        if gui_data.is_connected
            try
                lqr_action = [gui_data.lqr_gains, 0];
                gui_data.ctrl.step(gui_data.ctrl.CMD_SET_LQR_GAINS, lqr_action);
                fprintf('Ganancias LQR enviadas al Arduino\n');
            catch ME
                fprintf('Error enviando ganancias LQR: %s\n', ME.message);
            end
        end
        
        fprintf('Ganancias LQR calculadas: %s\n', k_str);
        
    catch ME
        msgbox(['Error calculando LQR: ' ME.message], 'Error', 'error');
    end
end

function start_control(~, ~)
    global gui_data;
    
    if ~gui_data.is_connected
        msgbox('Debe conectarse al Arduino primero', 'Error', 'error');
        return;
    end
    
    base_action = [0, 0, 0, 0, 0, 0];
    
    % Validar y enviar parámetros según controlador
    if gui_data.current_controller == 0 % PID
        try
            for i = 1:3
                gui_data.pid_gains(i) = str2double(get(gui_data.pid_edits(i), 'String'));
            end
            
            if any(isnan(gui_data.pid_gains))
                msgbox('Las ganancias PID deben ser números válidos', 'Error', 'error');
                return;
            end
            
            pid_action = [gui_data.pid_gains, 0, 0, 0];
            gui_data.ctrl.step(gui_data.ctrl.CMD_SET_PID_GAINS, pid_action);
            pause(0.1);
            
        catch
            msgbox('Error en los parámetros PID', 'Error', 'error');
            return;
        end
    else % LQR
        if all(gui_data.lqr_gains == 0)
            msgbox('Debe calcular las ganancias LQR primero', 'Error', 'error');
            return;
        end
    end
    
    % Limpiar datos y inicializar
    clear_data();
    gui_data.swing_start_time = tic;
    gui_data.upright_achieved = false;
    gui_data.max_theta_achieved = 0;
    
    % Enviar selección de controlador y comando de inicio
    controller_action = base_action;
    controller_action(1) = gui_data.current_controller;
    gui_data.ctrl.step(gui_data.ctrl.CMD_SELECT_CONTROLLER, controller_action);
    pause(0.1);
    
    gui_data.ctrl.step(gui_data.ctrl.CMD_START_CONTROL, base_action);
    
    % Actualizar estado
    gui_data.is_running = true;
    set(gui_data.start_btn, 'Enable', 'off');
    set(gui_data.stop_btn, 'Enable', 'on');
    set(gui_data.swing_status, 'String', 'Estado: Iniciando Swing-up', 'ForegroundColor', [0, 0, 0.8]);
    
    % Iniciar timer
    start(gui_data.timer);
    
    controller_name = {'PID', 'LQR'};
    fprintf('=== SWING-UP INICIADO ===\n');
    fprintf('Controlador: %s\n', controller_name{gui_data.current_controller + 1});
    fprintf('Iniciando impulsos con motor stepper\n');
    fprintf('Objetivo: Péndulo invertido (180°)\n');
end

function stop_control(~, ~)
    global gui_data;
    
    if gui_data.is_connected
        base_action = [0, 0, 0, 0, 0, 0];
        gui_data.ctrl.step(gui_data.ctrl.CMD_STOP_CONTROL, base_action);
    end
    
    % Actualizar estado
    gui_data.is_running = false;
    set(gui_data.start_btn, 'Enable', 'on');
    set(gui_data.stop_btn, 'Enable', 'off');
    set(gui_data.swing_status, 'String', 'Estado: Sistema Detenido', 'ForegroundColor', [0.8, 0.5, 0]);
    
    % Detener timer
    if isvalid(gui_data.timer)
        stop(gui_data.timer);
    end
    
    fprintf('=== CONTROL DETENIDO ===\n');
end

function set_home(~, ~)
    global gui_data;
    
    if gui_data.is_connected && ~gui_data.is_running
        base_action = [0, 0, 0, 0, 0, 0];
        gui_data.ctrl.step(gui_data.ctrl.CMD_SET_HOME, base_action);
        fprintf('Posición home establecida\n');
    end
end

function save_data(~, ~)
    global gui_data;
    
    if isempty(gui_data.sample_data)
        msgbox('No hay datos para guardar', 'Información', 'info');
        return;
    end
    
    try
        % Crear estructura de datos completa
        data_struct = struct();
        data_struct.sample = gui_data.sample_data;
        data_struct.time = gui_data.time_data;
        data_struct.theta = gui_data.theta_data;
        data_struct.phi = gui_data.phi_data;
        data_struct.dtheta = gui_data.dtheta_data;
        data_struct.dphi = gui_data.dphi_data;
        data_struct.swing_impulse = gui_data.swing_impulse_data;
        data_struct.energy_error = gui_data.energy_error_data;
        data_struct.status = gui_data.status_data;
        data_struct.controller = gui_data.current_controller;
        data_struct.max_theta_achieved = gui_data.max_theta_achieved;
        data_struct.upright_achieved = gui_data.upright_achieved;
        
        if gui_data.current_controller == 0
            data_struct.pid_gains = gui_data.pid_gains;
        else
            data_struct.lqr_gains = gui_data.lqr_gains;
        end
        
        % Guardar archivo
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
        controller_name = {'PID', 'LQR'};
        filename = sprintf('pendulum_swingup_%s_%s.mat', controller_name{gui_data.current_controller + 1}, timestamp);
        
        save(filename, 'data_struct');
        
        % Guardar gráficas
        figure_copy = figure('Visible', 'off', 'Position', [0, 0, 1400, 900]);
        copyobj(allchild(gui_data.plot_panel), figure_copy);
        
        img_filename = sprintf('pendulum_swingup_%s_%s.png', controller_name{gui_data.current_controller + 1}, timestamp);
        saveas(figure_copy, img_filename);
        close(figure_copy);
        
        msgbox(sprintf('Datos guardados:\n%s\n%s', filename, img_filename), 'Guardado', 'info');
        
    catch ME
        msgbox(['Error guardando datos: ' ME.message], 'Error', 'error');
    end
end

function update_display(~, ~)
    global gui_data;
    
    if ~gui_data.is_connected || isempty(gui_data.ctrl)
        return;
    end
    
    try
        base_action = [0, 0, 0, 0, 0, 0];
        
        % Usar comando que no interfiera con el control
        resp = gui_data.ctrl.step(gui_data.ctrl.CMD_MOVE_BY, base_action);
        
        if ~isempty(resp) && isstruct(resp) && isfield(resp, 'observation') && isfield(resp, 'timestamp')
            timestamp = resp.timestamp;
            obs = resp.observation;
            status = resp.status;
            
            % VALIDACIÓN MÁS ESTRICTA
            if ~isempty(obs) && length(obs) >= 6 && all(isfinite(obs(1:6)))
                % Calcular tiempo relativo
                if isempty(gui_data.time_data)
                    gui_data.start_timestamp = timestamp;
                    current_time = 0;
                else
                    current_time = (timestamp - gui_data.start_timestamp) / 1000.0;
                end
                
                gui_data.sample_counter = gui_data.sample_counter + 1;
                
                % Agregar datos ASEGURANDO que todos los arrays tengan la misma longitud
                gui_data.sample_data(end+1) = gui_data.sample_counter;
                gui_data.time_data(end+1) = current_time;
                gui_data.theta_data(end+1) = obs(1);  % Ángulo péndulo
                gui_data.phi_data(end+1) = obs(2);    % Ángulo rotor (stepper)
                gui_data.dtheta_data(end+1) = obs(3); % Velocidad péndulo
                gui_data.dphi_data(end+1) = obs(4);   % Velocidad rotor
                gui_data.swing_impulse_data(end+1) = obs(5); % Ángulo impulso
                gui_data.energy_error_data(end+1) = obs(6);  % Energía o error
                gui_data.status_data(end+1) = status;
                
                % VERIFICAR SINCRONIZACIÓN de arrays
                lengths = [length(gui_data.sample_data), length(gui_data.time_data), ...
                          length(gui_data.theta_data), length(gui_data.phi_data), ...
                          length(gui_data.dtheta_data), length(gui_data.dphi_data), ...
                          length(gui_data.swing_impulse_data), length(gui_data.energy_error_data), ...
                          length(gui_data.status_data)];
                
                if ~all(lengths == lengths(1))
                    fprintf('ADVERTENCIA: Arrays desincronizados: %s\n', mat2str(lengths));
                    % Truncar todos al tamaño mínimo
                    min_length = min(lengths);
                    fields = {'sample_data', 'time_data', 'theta_data', 'phi_data', ...
                             'dtheta_data', 'dphi_data', 'swing_impulse_data', 'energy_error_data', 'status_data'};
                    for i = 1:length(fields)
                        gui_data.(fields{i}) = gui_data.(fields{i})(1:min_length);
                    end
                end
                
                % Actualizar información de swing-up
                update_swing_info(obs, status, current_time);
                
                % Limitar puntos para rendimiento
                if length(gui_data.sample_data) > gui_data.max_points
                    fields = {'sample_data', 'time_data', 'theta_data', 'phi_data', ...
                             'dtheta_data', 'dphi_data', 'swing_impulse_data', 'energy_error_data', 'status_data'};
                    for i = 1:length(fields)
                        gui_data.(fields{i})(1) = [];
                    end
                end
                
                % Actualizar gráficas
                update_plots();
                
                % Debug cada 25 muestras (más frecuente para detectar problemas)
                if mod(gui_data.sample_counter, 1) == 0
                    fprintf('Muestra %d: θ=%.1f°, φ=%.1f°, Status=%d, Arrays=%d\n', ...
                        gui_data.sample_counter, obs(1), obs(2), status, length(gui_data.sample_data));
                end
                
            else
                fprintf('Datos inválidos recibidos: obs length=%d, finite=%d\n', ...
                    length(obs), sum(isfinite(obs)));
            end
        else
            % Debug cuando no se reciben datos
            if mod(gui_data.sample_counter, 100) == 0
                fprintf('Sin datos válidos recibidos (intento %d)\n', gui_data.sample_counter);
            end
        end
        
    catch ME
        % Solo reportar errores críticos, pero más información
        if toc(gui_data.last_successful_read) > 2 % Reducir timeout
            fprintf('Error comunicación: %s (línea %d)\n', ME.message, ME.stack(1).line);
            gui_data.last_successful_read = tic;
        end
    end
end

function update_swing_info(obs, status, current_time)
    global gui_data;
    
    theta = obs(1);
    energy = obs(6);
    
    % Actualizar máximo ángulo alcanzado
    angle_from_bottom = theta;
    if angle_from_bottom > 180
        angle_from_bottom = 360 - angle_from_bottom;
    end
    if angle_from_bottom > gui_data.max_theta_achieved
        gui_data.max_theta_achieved = angle_from_bottom;
    end
    
    % Actualizar displays
    set(gui_data.time_display, 'String', sprintf('Tiempo: %.1f s', current_time));
    set(gui_data.max_angle_display, 'String', sprintf('Máx θ: %.1f°', gui_data.max_theta_achieved));
    
    % Actualizar según estado
    switch status
        case gui_data.STATUS_OK
            set(gui_data.swing_status, 'String', 'Estado: Sistema Listo', 'ForegroundColor', [0, 0.6, 0]);
            set(gui_data.phase_display, 'String', 'Fase: Idle');
            set(gui_data.energy_display, 'String', 'Energía: 0.0');
            
        case gui_data.STATUS_STP_MOVING
            set(gui_data.swing_status, 'String', 'Estado: Motor en Movimiento', 'ForegroundColor', [0.6, 0.6, 0]);
            set(gui_data.phase_display, 'String', 'Fase: Posicionando');
            
        case gui_data.STATUS_SWING_UP
            set(gui_data.swing_status, 'String', 'Estado: SWING-UP ACTIVO', 'ForegroundColor', [1, 0.5, 0]);
            set(gui_data.energy_display, 'String', sprintf('Energía: %.2f', energy));
            
            % Determinar fase de swing-up
            impulse_angle = obs(5);
            if abs(impulse_angle) > 1
                set(gui_data.phase_display, 'String', sprintf('Fase: Impulso %.1f°', impulse_angle));
            else
                set(gui_data.phase_display, 'String', 'Fase: Espera');
            end
            
        case gui_data.STATUS_CONTROL_ACTIVE
            controller_name = {'PID', 'LQR'};
            set(gui_data.swing_status, 'String', sprintf('Estado: Control %s Activo', controller_name{gui_data.current_controller + 1}), 'ForegroundColor', [0, 0, 0.8]);
            set(gui_data.phase_display, 'String', 'Fase: Estabilizando');
            set(gui_data.energy_display, 'String', sprintf('Error: %.3f', energy));
            
        case gui_data.STATUS_UPRIGHT_ACHIEVED
            if ~gui_data.upright_achieved
                gui_data.upright_achieved = true;
                fprintf('*** ¡PÉNDULO INVERTIDO LOGRADO! ***\n');
                fprintf('Tiempo: %.1f s\n', current_time);
                fprintf('Ángulo máximo alcanzado: %.1f°\n', gui_data.max_theta_achieved);
            end
            set(gui_data.swing_status, 'String', '🎉 ¡PÉNDULO INVERTIDO LOGRADO!', 'ForegroundColor', [0, 0.8, 0]);
            set(gui_data.phase_display, 'String', 'Fase: ¡Éxito!');
            set(gui_data.energy_display, 'String', sprintf('Error: %.3f', energy));
    end
end

function update_plots()
    global gui_data;
    
    if isempty(gui_data.sample_data) || length(gui_data.sample_data) < 2
        return;
    end
    
    try
        n_points = length(gui_data.sample_data);
        samples = gui_data.sample_data;
        
        % Determinar ventana de visualización (últimas 300 muestras para mejor visualización)
        if n_points > 300
            idx_start = n_points - 299;
            idx_end = n_points;
            x_data = samples(idx_start:idx_end);
        else
            idx_start = 1;
            idx_end = n_points;
            x_data = samples;
        end
        
        % VALIDAR QUE TENEMOS DATOS VÁLIDOS
        if isempty(x_data) || length(x_data) < 2
            return;
        end
        
        % 1. Estado del sistema
        if length(gui_data.status_data) >= idx_end
            y_data = gui_data.status_data(idx_start:idx_end);
            if ~isempty(y_data) && length(y_data) == length(x_data)
                set(gui_data.line1, 'XData', x_data, 'YData', y_data);
                xlim(gui_data.ax1, [min(x_data)-1, max(x_data)+1]);
                % Asegurar que el eje Y muestre todos los estados
                ylim(gui_data.ax1, [-0.5, 4.5]);
                set(gui_data.ax1, 'YTick', 0:4, 'YTickLabel', {'Idle', 'Swing', 'Control', 'Stop', 'Upright'});
            end
        end
        
        % 2. Ángulo del péndulo con setpoint
        if length(gui_data.theta_data) >= idx_end
            theta_data = gui_data.theta_data(idx_start:idx_end);
            if ~isempty(theta_data) && length(theta_data) == length(x_data)
                setpoint_data = 180 * ones(size(theta_data));
                set(gui_data.line2a, 'XData', x_data, 'YData', setpoint_data);
                set(gui_data.line2b, 'XData', x_data, 'YData', theta_data);
                xlim(gui_data.ax2, [min(x_data)-1, max(x_data)+1]);
                % Ajustar límites Y dinámicamente
                theta_min = min(theta_data);
                theta_max = max(theta_data);
                if theta_max > theta_min
                    margin = (theta_max - theta_min) * 0.1;
                    ylim(gui_data.ax2, [max(0, theta_min - margin), min(360, theta_max + margin)]);
                else
                    ylim(gui_data.ax2, [0, 360]);
                end
            end
        end
        
        % 3. Ángulo del rotor (stepper)
        if length(gui_data.phi_data) >= idx_end
            phi_data = gui_data.phi_data(idx_start:idx_end);
            if ~isempty(phi_data) && length(phi_data) == length(x_data)
                set(gui_data.line3, 'XData', x_data, 'YData', phi_data);
                xlim(gui_data.ax3, [min(x_data)-1, max(x_data)+1]);
                % Ajustar límites Y dinámicamente
                phi_min = min(phi_data);
                phi_max = max(phi_data);
                if phi_max > phi_min
                    phi_range = phi_max - phi_min;
                    margin = max(phi_range * 0.1, 10); % Margen mínimo de 10 grados
                    ylim(gui_data.ax3, [phi_min - margin, phi_max + margin]);
                else
                    ylim(gui_data.ax3, [-180, 180]); % Rango por defecto
                end
            end
        end
        
        % 4. Velocidades angulares
        if length(gui_data.dtheta_data) >= idx_end && length(gui_data.dphi_data) >= idx_end
            dtheta_data = gui_data.dtheta_data(idx_start:idx_end);
            dphi_data = gui_data.dphi_data(idx_start:idx_end);
            if ~isempty(dtheta_data) && ~isempty(dphi_data) && ...
               length(dtheta_data) == length(x_data) && length(dphi_data) == length(x_data)
                set(gui_data.line4a, 'XData', x_data, 'YData', dtheta_data);
                set(gui_data.line4b, 'XData', x_data, 'YData', dphi_data);
                xlim(gui_data.ax4, [min(x_data)-1, max(x_data)+1]);
                % Ajustar límites Y para velocidades
                all_vel_data = [dtheta_data(:); dphi_data(:)];
                vel_min = min(all_vel_data);
                vel_max = max(all_vel_data);
                if vel_max > vel_min
                    vel_range = vel_max - vel_min;
                    margin = max(vel_range * 0.1, 50); % Margen mínimo de 50 deg/s
                    ylim(gui_data.ax4, [vel_min - margin, vel_max + margin]);
                else
                    ylim(gui_data.ax4, [-500, 500]); % Rango por defecto
                end
            end
        end
        
        % 5. Impulso de swing-up
        if length(gui_data.swing_impulse_data) >= idx_end
            impulse_data = gui_data.swing_impulse_data(idx_start:idx_end);
            if ~isempty(impulse_data) && length(impulse_data) == length(x_data)
                set(gui_data.line5, 'XData', x_data, 'YData', impulse_data);
                xlim(gui_data.ax5, [min(x_data)-1, max(x_data)+1]);
                % Ajustar límites Y para impulsos
                imp_min = min(impulse_data);
                imp_max = max(impulse_data);
                if imp_max > imp_min && (imp_max - imp_min) > 1
                    margin = max((imp_max - imp_min) * 0.1, 5);
                    ylim(gui_data.ax5, [imp_min - margin, imp_max + margin]);
                else
                    ylim(gui_data.ax5, [-50, 50]); % Rango por defecto
                end
            end
        end
        
        % 6. Energía/Error
        if length(gui_data.energy_error_data) >= idx_end
            energy_data = gui_data.energy_error_data(idx_start:idx_end);
            if ~isempty(energy_data) && length(energy_data) == length(x_data)
                set(gui_data.line6, 'XData', x_data, 'YData', energy_data);
                xlim(gui_data.ax6, [min(x_data)-1, max(x_data)+1]);
                % Ajustar límites Y para energía/error
                energy_min = min(energy_data);
                energy_max = max(energy_data);
                if energy_max > energy_min
                    energy_range = energy_max - energy_min;
                    margin = max(energy_range * 0.1, 1);
                    ylim(gui_data.ax6, [energy_min - margin, energy_max + margin]);
                else
                    ylim(gui_data.ax6, [-10, 10]); % Rango por defecto
                end
            end
        end
        
        % Forzar actualización inmediata de todas las gráficas
        drawnow;
        
        % Debug: Imprimir información cada 50 muestras
        if mod(n_points, 50) == 0
            fprintf('Graficando: %d puntos, theta=%.1f°, phi=%.1f°\n', ...
                n_points, gui_data.theta_data(end), gui_data.phi_data(end));
        end
        
    catch ME
        fprintf('Error actualizando gráficas: %s\n', ME.message);
        fprintf('Stack trace:\n');
        for i = 1:length(ME.stack)
            fprintf('  %s (línea %d)\n', ME.stack(i).name, ME.stack(i).line);
        end
    end
end

function clear_data()
    global gui_data;
    
    % Limpiar todos los arrays de datos
    fields = {'time_data', 'sample_data', 'theta_data', 'phi_data', 'dtheta_data', ...
             'dphi_data', 'swing_impulse_data', 'energy_error_data', 'status_data'};
    
    for i = 1:length(fields)
        gui_data.(fields{i}) = [];
    end
    
    gui_data.start_timestamp = [];
    gui_data.last_timestamp = 0;
    gui_data.sample_counter = 0;
    gui_data.max_theta_achieved = 0;
    gui_data.upright_achieved = false;
end

function disconnect_arduino()
    global gui_data;
    
    if gui_data.is_running
        stop_control();
    end
    
    if gui_data.is_connected && ~isempty(gui_data.ctrl)
        try
            gui_data.ctrl.close();
        catch ME
            fprintf('Error cerrando conexión: %s\n', ME.message);
        end
        gui_data.is_connected = false;
    end
    
    % Actualizar interfaz
    set(gui_data.connect_btn, 'String', 'Conectar', 'BackgroundColor', [0, 0.8, 0]);
    set(gui_data.status_text, 'String', 'Estado: Desconectado', 'ForegroundColor', [0.8, 0, 0]);
    set([gui_data.start_btn, gui_data.stop_btn, gui_data.home_btn, gui_data.save_btn], 'Enable', 'off'); 
    
    fprintf('=== DESCONECTADO DEL ARDUINO ===\n');
end

function close_gui(~, ~)
    global gui_data;
    
    % Detener timer
    if isfield(gui_data, 'timer') && isvalid(gui_data.timer)
        try
            stop(gui_data.timer);
            delete(gui_data.timer);
        catch
            % Ignorar errores
        end
    end
    
    % Desconectar Arduino
    disconnect_arduino();
    
    % Cerrar figura
    if isfield(gui_data, 'fig') && isvalid(gui_data.fig)
        delete(gui_data.fig);
    end
    
    fprintf('=== GUI CERRADA ===\n');
end