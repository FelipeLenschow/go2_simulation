import csv
import math

def extract_column(file_path, target_header):
    try:
        with open(file_path, mode='r') as file:
            csv_reader = csv.reader(file)
            headers = next(csv_reader)

            if target_header in headers:
                target_index = headers.index(target_header)
                print(f"Header '{target_header}' found at index {target_index}.")

                column_data = [row[target_index] if len(row) > target_index else '' for row in csv_reader]
                return column_data
            else:
                print(f"Header '{target_header}' not found in the file.")
                return None
    except FileNotFoundError:
        print("The specified file does not exist.")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

def forward_fill(data):
    filled = []
    last_valid = None
    for value in data:
        if value.strip() == '':
            filled.append(last_valid if last_valid is not None else '0.0')
        else:
            last_valid = value
            filled.append(value)
    return filled

def align_and_fill(reference, lowstate):
    # Encontrar o índice do primeiro valor não vazio na referência
    first_valid_index = next((i for i, val in enumerate(reference) if val.strip() != ''), None)

    if first_valid_index is None:
        print("Erro: Nenhum valor válido encontrado na referência.")
        return [], []

    # Cortar ambos os sinais a partir do primeiro valor válido
    aligned_ref = reference[first_valid_index:]
    aligned_low = lowstate[first_valid_index:]

    # Preencher os valores vazios na referência
    aligned_ref = forward_fill(aligned_ref)
    aligned_low = forward_fill(aligned_low)  # opcional, se quiser evitar vazios aqui também

    return aligned_ref, aligned_low

def clean_data_for_float(data):
    cleaned = []
    for val in data:
        try:
            cleaned.append(float(val))
        except:
            cleaned.append(0.0)  # ou `continue` se quiser ignorar esse ponto
    return cleaned

def calculate_rms_error(data1, data2):
    if len(data1) != len(data2):
        min_len = min(len(data1), len(data2))
        print(f"Atenção: vetores com tamanhos diferentes. Truncando para {min_len} elementos.")
        data1 = data1[:min_len]
        data2 = data2[:min_len]

    data1 = clean_data_for_float(data1)
    data2 = clean_data_for_float(data2)

    squared_diffs = [(a - b) ** 2 for a, b in zip(data1, data2)]
    mean_squared_diff = sum(squared_diffs) / len(squared_diffs)
    rms = math.sqrt(mean_squared_diff)
    return rms

# Caminho do arquivo
file_path = "PDCOMPG_dados.csv"

# Headers
target_header_lowstate = "/lowstate/motor_state[0]/q"
target_header_reference = "/go2_jointcontroller/JointControllerReferences/motor_cmd[0]/q"

# Extração
column_data1 = extract_column(file_path, target_header_lowstate)
column_data2 = extract_column(file_path, target_header_reference)

# Alinhamento e preenchimento
if column_data1 and column_data2:
    column_data2, column_data1 = align_and_fill(column_data2, column_data1)

# Cálculo da RMS
if column_data1 is not None and column_data2 is not None:
    rms_error = calculate_rms_error(column_data1, column_data2)
    if rms_error is not None:
        print(f"\nRMS do erro entre os sinais: {rms_error}")
    else:
        print("\nNão foi possível calcular a RMS devido a erro de conversão.")
else:
    print("\nNão foi possível calcular a RMS por falta de dados.")