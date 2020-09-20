#include "bme280.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <time.h>

#define CSV_PATH "data.csv"
#define CLOCK_IN_SECONDS 10 //tempo para leitura do csv

/* Armazena dados da conexão com o device */
struct identificador
{
    uint8_t enderecoDoDispositivo;
    int8_t descritorDoArquivo;
};

struct sensorData {
    float umidade;
    float pressao;
    float temperatura;
};

void user_delay_us(uint32_t period, void *intf_ptr);
void handle_sensor_data(struct bme280_data *comp_data, int* count, struct sensorData* sensorData);
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev, struct sensorData* sensorData);
void initCSV();
void saveInFile(struct sensorData* sensorData);

int main(int argc, char* argv[]){
    struct bme280_dev dispositivo;
    struct identificador user_id;
    struct sensorData sensorData[10];

    /* Variable to define the result */
    int8_t rslt = BME280_OK;

    initCSV();
    if ((user_id.descritorDoArquivo = open("/dev/i2c-1", O_RDWR)) < 0)
    {
        fprintf(stderr, "Erro ao abrir o arquivo /dev/i2c-1\n");
        exit(1);
    }

    if (ioctl(user_id.descritorDoArquivo, I2C_SLAVE, 0x76) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        exit(1);
    }

    user_id.enderecoDoDispositivo = BME280_I2C_ADDR_PRIM;

    dispositivo.intf = BME280_I2C_INTF;
    dispositivo.read = user_i2c_read;
    dispositivo.write = user_i2c_write;
    dispositivo.delay_us = user_delay_us;

    /* Update interface pointer with the structure that contains both device address and file descriptor */
    dispositivo.intf_ptr = &user_id;

    /* Initialize the bme280 */
    rslt = bme280_init(&dispositivo);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to initialize the device (code %+d).\n", rslt);
        exit(1);
    }

    rslt = stream_sensor_data_normal_mode(&dispositivo, sensorData);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to stream sensor data (code %+d).\n", rslt);
        exit(1);
    }

    return 0;
}

void initCSV(){
    FILE* csv = fopen(CSV_PATH, "w+");
    fwrite("Data,Temperatura,Umidade,Pressão\n" , 1 , strlen("Data,Temperatura,Umidade,Pressão\n") , csv);
    fclose(csv);
}

void saveInFile(struct sensorData* sensorData){
    float somaTemperatura = 0, somaUmidade = 0, somaPressao = 0;
    for(int i = 0; i < CLOCK_IN_SECONDS; i++){
        somaTemperatura += sensorData[i].temperatura;
        somaUmidade += sensorData[i].umidade;
        somaPressao =+ sensorData[i].pressao;
    }
    float mediaTemperatura = somaTemperatura / CLOCK_IN_SECONDS;
    float mediaUmidade = somaUmidade / CLOCK_IN_SECONDS;
    float mediaPressao = somaPressao / CLOCK_IN_SECONDS;
    
    time_t timer;
    char dateTime[26];
    struct tm* tm_info;

    timer = time(NULL);
    tm_info = localtime(&timer);

    strftime(dateTime, 26, "%Y-%m-%d %H:%M:%S", tm_info);

    char linha[100];
    sprintf(linha, "%s,%.2f,%.2f,%.2f\n", dateTime, mediaTemperatura, mediaUmidade, mediaPressao);
    printf("%s - Salvando linha da média das ultimas leituras: %s\n\n", dateTime, linha);
    FILE* csv = fopen(CSV_PATH, "a+");
    fwrite(linha, 1, strlen(linha), csv);
    fclose(csv);
}


int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev, struct sensorData* sensorData)
{
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

    int count = 0;
	while (1) {
		dev->delay_us(1000000, dev->intf_ptr);
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
		handle_sensor_data(&comp_data, &count, sensorData);
	}

	return rslt;
}

void handle_sensor_data(struct bme280_data *comp_data, int* count, struct sensorData* sensorData)
{
    sensorData[*count].temperatura = comp_data->temperature;
    sensorData[*count].pressao = comp_data->pressure;
    sensorData[*count].umidade = comp_data->humidity;
    printf("lido(%d): t: %.2f, u: %.2f, p: %.2f\n", *count + 1, comp_data->temperature, comp_data->humidity, comp_data->pressure);
    if(*count == CLOCK_IN_SECONDS - 1){
        saveInFile(sensorData);
        *count = 0;
    } 
    else {
        *count = *count + 1;
    }
}

/*!
 * @brief This function for writing the sensor's registers through I2C bus.
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t *buf;
    struct identificador id;

    id = *((struct identificador *)intf_ptr);

    buf = malloc(len + 1);
    buf[0] = reg_addr;
    memcpy(buf + 1, data, len);
    if (write(id.descritorDoArquivo, buf, len + 1) < (uint16_t)len)
    {
        return BME280_E_COMM_FAIL;
    }

    free(buf);

    return BME280_OK;
}



/*!
 * @brief This function reading the sensor's registers through I2C bus.
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    struct identificador id;

    id = *((struct identificador *)intf_ptr);
    write(id.descritorDoArquivo, &reg_addr, 1);
    read(id.descritorDoArquivo, data, len);
    return 0;
}


void user_delay_us(uint32_t period, void *intf_ptr)
{
    usleep(period);
}