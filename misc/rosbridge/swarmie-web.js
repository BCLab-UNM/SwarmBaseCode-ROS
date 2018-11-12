
// Connecting to ROS
// -----------------
window.addEventListener('load', eventWindowLoaded, false);

function eventWindowLoaded () {
    runSwarmieWebsite();
}

function runSwarmieWebsite() {

    var urlParams = new URLSearchParams(window.location.search);

    var Name = urlParams.get('robotname');
    console.log(Name);
    var swarmie = new Swarmie(Name);

    document.title = Name;
};
