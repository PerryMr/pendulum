classdef ControlComms < handle
    %CONTROLCOMMS Clase para comunicación serial con Arduino/STM
    %   Esta clase maneja la comunicación bidireccional con el controlador
    %   del péndulo invertido mediante protocolo JSON sobre puerto serial
    
    properties (Access = private)
        serialObj          % Objeto puerto serial
        timeout            % Timeout en segundos
        debugLevel         % Nivel de debug
    end
    
    properties (Constant)
        % Códigos de estado
        STATUS_OK = 0
        STATUS_ERROR = 1
        
        % Niveles de debug
        DEBUG_NONE = 0
        DEBUG_ERROR = 1
        DEBUG_WARN = 2
        DEBUG_INFO = 3
        
        % Comandos para Arduino
        CMD_SET_HOME = 0            % Establecer posición Home
        CMD_MOVE_TO = 1             % Mover la posición absoluta
        CMD_MOVE_BY = 2             % Mover por incremento
        CMD_SET_STEP_MODE = 3       % Configurar modo de paso
        CMD_SELECT_CONTROLLER = 4   % Seleccionar controlador PID/LQR
        CMD_SET_PID_GAINS = 5       % Configurar ganancias PID
        CMD_SET_LQR_GAINS = 6       % Configurar ganancias LQR
        CMD_START_CONTROL = 7       % Iniciar con el control
        CMD_STOP_CONTROL = 8        % Detener el control
        
        % Estados del sistema
        STATUS_STP_MOVING = 1       % Motor en movimiento
        STATUS_EXCITATION = 2       % En excitación
        STATUS_CONTROL_ACTIVE = 3   % Control activo
        STATUS_UPRIGHT_ACHIEVED = 4 % Péndulo invertido estabilizado
        
        % Controladores
        CONTROLLER_PID = 0
        CONTROLLER_LQR = 1
        
        % Claves JSON para transmisión
        TX_KEY_ACTION = 'action'
        TX_KEY_COMMAND = 'command'
        
        % Claves JSON para recepción
        RX_KEY_STATUS = 'status'
        RX_KEY_TIMESTAMP = 'timestamp'
        RX_KEY_TERMINATED = 'terminated'
        RX_KEY_OBSERVATION = 'observation'
    end
    
    methods
        function obj = ControlComms(timeout, debugLevel)
            %CONTROLCOMMS Constructor
            %   timeout - Tiempo de espera en segundos (default: 1.0)
            %   debugLevel - Nivel de debug (default: DEBUG_NONE)
            
            if nargin < 1
                timeout = 1.0;
            end
            if nargin < 2
                debugLevel = obj.DEBUG_NONE;
            end
            
            obj.timeout = timeout;
            obj.debugLevel = debugLevel;
            obj.serialObj = [];
        end
        
        function delete(obj)
            %DELETE Destructor - cierra puerto serial
            obj.close();
        end
        
        function portList = getSerialList(obj)
            %GETSERIALLIST Obtiene lista de puertos seriales disponibles
            %   Retorna cell array con información de puertos
            
            try
                portInfo = serialportlist("available");
                portList = cellstr(portInfo);
            catch
                % Para versiones anteriores de MATLAB
                try
                    portInfo = instrhwinfo('serial');
                    if isfield(portInfo, 'AvailableSerialPorts')
                        portList = portInfo.AvailableSerialPorts;
                    else
                        portList = {};
                    end
                catch
                    portList = {};
                end
            end
        end
        
        function status = connect(obj, port, baudRate)
            %CONNECT Conecta al puerto serial especificado
            %   port - Nombre del puerto (ej: 'COM10')
            %   baudRate - Velocidad de comunicación (default: 115200)
            %   Retorna STATUS_OK o STATUS_ERROR
            
            if nargin < 3
                baudRate = 115200;
            end
            
            % Cerrar conexión previa si existe
            obj.close();
            
            try
                % Crear objeto serial
                if exist('serialport', 'file') == 2
                    % MATLAB R2019b o posterior
                    obj.serialObj = serialport(port, baudRate, ...
                        'Timeout', obj.timeout, ...
                        'DataBits', 8, ...
                        'Parity', 'none', ...
                        'StopBits', 1, ...
                        'FlowControl', 'none');
                else
                    % Versiones anteriores de MATLAB
                    obj.serialObj = serial(port, ...
                        'BaudRate', baudRate, ...
                        'Timeout', obj.timeout, ...
                        'DataBits', 8, ...
                        'Parity', 'none', ...
                        'StopBits', 1, ...
                        'FlowControl', 'none', ...
                        'Terminator', 'LF');
                    fopen(obj.serialObj);
                end
                
                status = obj.STATUS_OK;
                
                if obj.debugLevel >= obj.DEBUG_INFO
                    fprintf('Conectado a %s a %d baudios\n', port, baudRate);
                end
                
            catch ME
                if obj.debugLevel >= obj.DEBUG_ERROR
                    fprintf('Error conectando a puerto serial: %s\n', ME.message);
                end
                status = obj.STATUS_ERROR;
            end
        end
        
        function close(obj)
            %CLOSE Cierra la conexión serial
            
            if ~isempty(obj.serialObj)
                try
                    if exist('serialport', 'file') == 2
                        % MATLAB R2019b o posterior
                        delete(obj.serialObj);
                    else
                        % Versiones anteriores
                        if strcmp(obj.serialObj.Status, 'open')
                            fclose(obj.serialObj);
                        end
                        delete(obj.serialObj);
                    end
                catch ME
                    if obj.debugLevel >= obj.DEBUG_WARN
                        fprintf('Advertencia cerrando puerto: %s\n', ME.message);
                    end
                end
                obj.serialObj = [];
            end
        end
        
        function result = step(obj, command, action)
            %STEP Envía comando y acciones, espera observación
            %   command - Comando entero a enviar
            %   action - Vector de acciones (números flotantes)
            %   Retorna struct con campos: status, timestamp, terminated, observation
            %   o empty si hay error
            
            result = [];
            
            if isempty(obj.serialObj)
                if obj.debugLevel >= obj.DEBUG_ERROR
                    fprintf('Error: Puerto serial no conectado\n');
                end
                return;
            end
            
            % Preparar mensaje JSON
            if isscalar(action)
                actionStr = sprintf('%.6f', action);
            else
                actionStr = sprintf('%.6f,', action);
                actionStr = actionStr(1:end-1); % Remover última coma
            end
            
            msg = sprintf('{"command":%d,"action":[%s]}', command, actionStr);
            
            try
                % Limpiar buffer de entrada antes de enviar
                if exist('serialport', 'file') == 2
                    if obj.serialObj.NumBytesAvailable > 0
                        flush(obj.serialObj);
                    end
                else
                    while obj.serialObj.BytesAvailable > 0
                        fread(obj.serialObj, obj.serialObj.BytesAvailable);
                    end
                end
                
                % Enviar mensaje
                if exist('serialport', 'file') == 2
                    % MATLAB R2019b o posterior
                    write(obj.serialObj, msg, 'string');
                else
                    % Versiones anteriores
                    fprintf(obj.serialObj, '%s\n', msg);
                end
                
                % Esperar respuesta con timeout mejorado
                response = '';
                attempts = 0;
                max_attempts = 10;
                
                while isempty(response) && attempts < max_attempts
                    try
                        if exist('serialport', 'file') == 2
                            response = readline(obj.serialObj);
                            response = char(response);
                        else
                            response = fgetl(obj.serialObj);
                        end
                        
                        if isempty(response) || (ischar(response) && strcmp(response, ''))
                            pause(0.01); % Breve pausa antes de reintentar
                            attempts = attempts + 1;
                        end
                    catch
                        attempts = attempts + 1;
                        pause(0.01);
                    end
                end
                
                if isempty(response) || attempts >= max_attempts
                    if obj.debugLevel >= obj.DEBUG_ERROR
                        fprintf('Timeout: No se recibió respuesta del Arduino\n');
                    end
                    return;
                end
                
                % Validar que la respuesta sea JSON válido
                if ~ischar(response) || isempty(strfind(response, '{'))
                    if obj.debugLevel >= obj.DEBUG_ERROR
                        fprintf('Respuesta inválida (no JSON): %s\n', response);
                    end
                    return;
                end
                
                % Parsear JSON
                data = jsondecode(response);
                
                % Crear estructura de resultado
                result = struct();
                
                % Verificar que los campos existen
                if isfield(data, obj.RX_KEY_STATUS)
                    result.status = data.(obj.RX_KEY_STATUS);
                else
                    result.status = obj.STATUS_ERROR;
                end
                
                if isfield(data, obj.RX_KEY_TIMESTAMP)
                    result.timestamp = data.(obj.RX_KEY_TIMESTAMP);
                else
                    result.timestamp = 0;
                end
                
                if isfield(data, obj.RX_KEY_TERMINATED)
                    result.terminated = data.(obj.RX_KEY_TERMINATED);
                else
                    result.terminated = false;
                end
                
                if isfield(data, obj.RX_KEY_OBSERVATION)
                    result.observation = data.(obj.RX_KEY_OBSERVATION);
                else
                    result.observation = [];
                end
                
            catch ME
                if obj.debugLevel >= obj.DEBUG_ERROR
                    fprintf('Error en comunicación: %s\n', ME.message);
                    if exist('response', 'var') && ~isempty(response)
                        fprintf('Respuesta recibida: %s\n', response);
                    end
                end
            end
        end
        
        function setTimeout(obj, timeout)
            %SETTIMEOUT Establece el timeout de comunicación
            %   timeout - Tiempo en segundos
            
            obj.timeout = timeout;
            if ~isempty(obj.serialObj)
                if exist('serialport', 'file') == 2
                    obj.serialObj.Timeout = timeout;
                else
                    obj.serialObj.Timeout = timeout;
                end
            end
        end
    end
end