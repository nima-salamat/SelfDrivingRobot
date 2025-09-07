import robot
import logging
import argparse

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

modes = ("city", "race")
def main():
    parser = argparse.ArgumentParser()
    
    subparser = parser.add_subparsers(dest="command", required=True)
    
    runserver_parser = subparser.add_parser("run", help="Run the robot")
    runserver_parser.add_argument("--mode", default="city", help="city/race", choices=modes)
    runserver_parser.add_argument("--display",action="store_true", help="show display")
    

    
    parser.add_argument(
        "--log", 
        default="DEBUG", 
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"], 
        help="Set logging level"
    )
    

    args = parser.parse_args()
    
    # set logging level
    logging.getLogger().setLevel(getattr(logging, args.log))
    
    match args.command:
        case "run":
            mode = args.mode
            display = args.display
            logging.info(f"Starting robot in {mode} mode . . . ")
            robot.start(mode, display)

if __name__ == "__main__":
    main()