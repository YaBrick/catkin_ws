import cv2

# Inicializa a câmera (indice 0 para a câmera padrão)
camera = cv2.VideoCapture(0)

try: 
    while camera.isOpened():
        # Lê um frame da câmera
        ret, frame = camera.read()
        if not ret:
            print("Nao foi possivel obter frame da camera")
            break
        
        # Salva a imagem em um arquivo
        cv2.imwrite("imagem_capturada.jpg", frame)

finally:
    # Libera a câmera
    print('Liberando recursos')
    camera.release()

