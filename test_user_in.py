from flymambo import *


def main():
    try:
        while True:
            input_command = input('Enter Mission Command: ')

            if input_command == 'UP':
                command_seq = ['UP']

            elif input_command == 'DOWN':
                command_seq = ['DOWN']

            elif input_command == 'DOWN':
                command_seq = ['DOWN']

            elif input_command == 'LEFT':
                command_seq = ['LEFT']

            elif input_command == 'RIGHT':
                command_seq = ['RIGHT']

            elif input_command == 'EXIT':
                print('Exiting Command...')
                exit(0)

            else:
                print('Invalid input...')

            print('Current Command Sequence: ', command_seq, flush=True)

    except KeyboardInterrupt:
        print('Exiting By User Interrupt...')

    return 0


if __name__ == '__main__':
    main()
