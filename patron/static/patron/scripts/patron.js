function dragElement(elmnt) {
    var x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    var imageSize = $('#image').get(0).getBoundingClientRect();
    var textBoxSize = $('#drag-item').get(0).getBoundingClientRect();

    elmnt.style.top = (imageSize.top + (imageSize.bottom - imageSize.top)/2) + "px";
    elmnt.style.left = (imageSize.left + (imageSize.right - imageSize.left)/2)+ "px";
    document.getElementById("drag-item-handle").onmousedown = dragMouseDown;

    function dragMouseDown(e) {
        e = e || window.event;
        e.preventDefault();
        // get the mouse cursor position at startup:
        x2 = e.clientX;
        y2 = e.clientY;
        document.onmouseup = closeDragElement;
        // call a function whenever the cursor moves:
        document.onmousemove = elementDrag;
    }

    function elementDrag(e) {
        e = e || window.event;
        e.preventDefault();
        // calculate the new cursor position:
        x1 = x2 - e.clientX;
        y1 = y2 - e.clientY;
        x2 = e.clientX;
        y2 = e.clientY;

        // set the element's new position:
        var top = (elmnt.offsetTop - y1);
        if(top > imageSize.top && top < imageSize.bottom - (textBoxSize.bottom - textBoxSize.top))
          elmnt.style.top = top + "px";

        var left = (elmnt.offsetLeft - x1);
        if(left > imageSize.left && left < imageSize.right - (textBoxSize.right - textBoxSize.left))
          elmnt.style.left = left + "px";
    }

    function closeDragElement() {
        /* stop moving when mouse button is released:*/
        document.onmouseup = null;
        document.onmousemove = null;
    }
}

$(document).ready(function () {
    $('.apply-btn').on('click', function () {
        var btnSize = $(this).get(0).getBoundingClientRect();
        var draggerSize = $('#drag-item').get(0).getBoundingClientRect();

        $(this).hide();
        $('.drag-btn').hide();

        document.getElementById("drag-item").style.top = draggerSize.top + (btnSize.bottom - btnSize.top) + 'px';
        $('#drag-item').css({'background-color': "rgba(255, 255, 255, 0)"});
    });
});