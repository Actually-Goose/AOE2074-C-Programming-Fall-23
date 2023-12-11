/* 
 * PocketAudio file that recognizes live speech and
 * converts it into text.
 * Author: Bryan Duong and Ayden De Guzman
 */

/**
 * @example pocketAudio
 * @brief Speech recognition with live audio input and endpointing.
 *
 * This file shows how to use PocketSphinx with microphone input using
 * PortAudio (v19 and above).
 * 
 * Compile command:
 * 
 *      gcc -o pocketAudio pocketAudio.c $(pkg-config --libs --cflags pocketsphinx portaudio-2.0)
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <portaudio.h>
#include <pocketsphinx.h>
#include <signal.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <time.h>
/**
 * This variable is used as a flag to signal when the program should terminate. 
 * It is set to 1 when a signal (e.g., SIGINT) is received.
*/
static int global_done = 0;
void filter(FILE *psTxt);
static void catchSig(int signum);
char* extractWord(const char* inputString, int wordIndex);

/**
 * This program captures live audio, recognizes speech using PocketSphinx, 
 * and writes the recognized text to a file. It handles speech 
 * start and end events with the help of PortAudio and PocketSphinx libraries.
 */
int main(int argc, char *argv[]) {
    /**
     * The code initializes PortAudio, PocketSphinx, and related data structures.
     * It sets up a configuration for PocketSphinx and initializes a decoder and an endpointer.
     * A PortAudio stream (stream) is created to capture real time audio and output to device.
     * Memory is allocated for the audio frame.
    */
    PaStream *stream;
    PaError err;
    ps_decoder_t *decoder;
    ps_config_t *config;
    ps_endpointer_t *ep;
    short *frame;
    size_t frame_size;

    config = ps_config_init(NULL);
    ps_default_search_args(config);

    /**
     * The PortAudio stream is opened with default settings (e.g., one channel, 16-bit PCM audio).
     * The stream is started, and a signal handler (catch_sig) is set up to handle SIGINT (Ctrl+C) signals.
    */
    signal(SIGINT, catchSig);
    err = Pa_Initialize();  
    decoder = ps_init(config);
    ep = ps_endpointer_init(0, 0.0, 0, 0, 0);
    frame_size = ps_endpointer_frame_size(ep);
    frame = malloc(frame_size * sizeof(frame[0]));
    err = Pa_OpenDefaultStream(&stream, 1, 0, paInt16, 
        ps_config_int(config, "samprate"),frame_size, NULL, NULL);
    err = Pa_StartStream(stream);
    FILE *output_file = fopen("recordedData.txt", "w");

    /**
     * Adds the key words that we want to spot in our speech
    */
    char name[10];
    ps_add_kws(decoder, name, "keyphrase.txt");
    ps_activate_search(decoder, name);
    
    bool intializeFlag = FALSE;
    bool takeOffFlag = FALSE;
    const char *prev_keyword;
    /**
     * The program enters a loop where it continuously reads audio frames from the microphone.
     * The endpointer (ps_endpointer_t) is used to detect speech start and end.
     * When speech is detected, the PocketSphinx decoder (ps_decoder_t) processes the raw audio.
     * Recognized speech hypotheses are then printed when speech ends for each segment
     */
    while (!global_done) {
        const int16 *speech;
        int prev_in_speech = ps_endpointer_in_speech(ep);
        err = Pa_ReadStream(stream, frame, frame_size);
        speech = ps_endpointer_process(ep, frame);
        // Speech recognition starts here
        if (speech != NULL) {
            const char *hyp;
            // Beginning of speech handling
            if (!prev_in_speech) {
                ps_start_utt(decoder);
            }

            // Process the raw audio with PocketSphinx
            ps_process_raw(decoder, speech, frame_size, FALSE, FALSE);

            // Handles speech and stores into hyp
            hyp = ps_get_hyp(decoder, NULL);
            if (!ps_endpointer_in_speech(ep)) {

                // Decodes speech
                ps_end_utt(decoder);
                if ((hyp = ps_get_hyp(decoder, NULL)) != NULL) {
                    // Processes commands if state is first initialized through saying "initialize"
                    if (intializeFlag){
                        // Takeoff must be the first command
                        if (!takeOffFlag){
                            if (strcmp(extractWord(hyp, 0), "takeoff") == 0){
                                printf("%s\n", extractWord(hyp, 0));
                                char yn[4];
                                printf("Is this the right command? (yes/no) ");
                                scanf("%s", yn);
                                if (strcmp(yn, "yes") == 0){
                                    printf("Command '%s' sent.\n", extractWord(hyp, 0));
                                    printf("Commencing flight run.\n");
                                    takeOffFlag = TRUE;
                                }
                                else{
                                    printf("Command not sent.\n");
                                }
                            }
                        } 
                        else {
                            /**
                             * Extracts first word of command from speech and proceeds to process the 
                             * needed words given the first word, tokenizes the string and 
                             * prints them off.
                            */
                            if (strcmp(hyp, "initialize") != 0 && strcmp(hyp, prev_keyword) != 0) {
                                if (strcmp(extractWord(hyp, 0), "land") == 0){
                                    printf("%s\n", extractWord(hyp, 0));
                                    char yn[4];
                                    printf("Is this the right command? (yes/no) ");
                                    scanf("%s", yn);
                                    if (strcmp(yn, "yes") == 0){
                                        printf("Command '%s' sent.\n", extractWord(hyp, 0));
                                        printf("Program terminated.\n");
                                        global_done = 1;
                                    }
                                    else{
                                        printf("Command not sent.\n");
                                    }
                                }
                                else if (strcmp(extractWord(hyp, 0), "go") == 0){
                                    printf("%s %s %s\n", extractWord(hyp, 0), 
                                        extractWord(hyp, 1), extractWord(hyp, 2));
                                    char yn[4];
                                    printf("Is this the right command? (yes/no) ");
                                    scanf("%s", yn);
                                    if (strcmp(yn, "yes") == 0){
                                        printf("Command '%s %s %s' sent.\n", extractWord(hyp, 0),
                                            extractWord(hyp, 1), extractWord(hyp, 2));
                                    }
                                    else{
                                        printf("Command not sent.\n");
                                    }
                                }
                                else if (strcmp(extractWord(hyp, 0), "way") == 0){
                                    printf("%s %s %s\n", extractWord(hyp, 0), 
                                        extractWord(hyp, 1), extractWord(hyp, 2));
                                    char yn[4];
                                    printf("Is this the right command? (yes/no) ");
                                    scanf("%s", yn);
                                    if (strcmp(yn, "yes") == 0){
                                        printf("Command '%s %s %s' sent.\n", extractWord(hyp, 0),
                                            extractWord(hyp, 1), extractWord(hyp, 2));
                                    }
                                    else{
                                        printf("Command not sent.\n");
                                    }
                                }
                                else if (strcmp(extractWord(hyp, 0), "self") == 0){
                                    printf("%s %s \n", extractWord(hyp, 0), 
                                        extractWord(hyp, 1));
                                    char yn[4];
                                    printf("Is this the right command? (yes/no) ");
                                    scanf("%s", yn);
                                    if (strcmp(yn, "yes") == 0){
                                        printf("Command '%s %s' sent.\n" 
                                        "Self destructing in t-minus...\n", extractWord(hyp, 0),
                                            extractWord(hyp, 1));

                                        // Self destructing
                                        int seconds = 5;
                                        while (seconds > 0){
                                            printf("%d...\n", seconds);
                                            clock_t stop = clock() + CLOCKS_PER_SEC;
                                            while (clock() < stop){

                                            }
                                            seconds--;
                                        }
                                        global_done = 1;
                                    }
                                    else{
                                        printf("Command not sent.\n");
                                    }
                                }
                            }
                        }
                    }
                    else{
                        if (strcmp(hyp, "initialize") == 0) {
                            printf("Initialized...\n");
                            intializeFlag = true;
                        }
                    }
                    // Puts data into file
                    //fprintf(output_file, "%s\n", hyp);
                    
                    //fflush(output_file);
                }
            }
        }
    }

    /**
     * Frees allocated memory and closes output file.
     */
    fclose(output_file);
    free(frame);
    ps_endpointer_free(ep);
    ps_free(decoder);
    ps_config_free(config);

    return 0;
}

/**
 * This function (catch_sig) is a signal handler. 
 * It sets the global_done flag to 1 when a signal is received.
 * @param signum
*/
static void catchSig(int signum) {
    (void)signum;
    global_done = 1;
}

/**
 * Parses text file
 * @param psTxt
*/
void filter(FILE *psTxt) {
    char line[200];
    char word[20];
    char initializer[] = "password";

    while (fgets(line, sizeof(line), psTxt) != NULL) {
        // Process each word in the line
        int i = 0;
        while (sscanf(line + i, "%19s", word) == 1) {
            if (strcmp(word, initializer) == 0){
                printf("Password: Pending Command\n");
            }
            // Move to the next word
            i += strlen(word);
            while (!isalnum(line[i]) && line[i] != '\0') {
                i++;
            }
        }
    }

    fclose(psTxt);
}

/**
 * Function to grab the nth/wordIndex word of the input string
 * @param inputString The input string
 * @param wordIndex The index of the word to extract (0-based)
 * @return The extracted word or NULL if not found
 */
char* extractWord(const char* inputString, int wordIndex) {
    size_t length = 0;
    const char* start = inputString;

    // Iterate through the input string until the desired word index is reached
    for (int i = 0; i <= wordIndex; i++) {
        /**
         * Skip trailing lines and move to next word.
        */
        start += strspn(start, " \t\n");
        length = strcspn(start, " \t\n");
        start += length + 1;
        start += strspn(start, " \t\n");
    }
    if (length == 0) {
        return NULL;
    }

    // Allocate memory for the extracted word
    char* extractedWord = (char*)malloc(length + 1);
    strncpy(extractedWord, start - length - 1, length);
    extractedWord[length] = '\0'; 

    return extractedWord;
}
